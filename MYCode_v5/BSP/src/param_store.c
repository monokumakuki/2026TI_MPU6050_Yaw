/******************************************************************
 * Flash 参数存储模块 - 实现
 *
 * MSPM0G3507 Flash 特性：
 *   - Flash word = 64位数据 + 8位 ECC = 72位
 *   - 写入地址必须 64-bit 对齐（低 3 位为 0）
 *   - 必须用 WithECCGenerated 系列 API，否则 ECC 码不更新
 *   - 扇区大小 1KB，擦除后全部为 0xFF（含 ECC）
 *
 * Flash 擦写流程：
 *   1. executeClearStatus              → 清除 Flash 状态寄存器
 *   2. unprotectSector(addr)           → 解锁目标扇区
 *   3. eraseMemory(addr, SECTOR)       → 扇区擦除为 0xFF
 *   4. waitForCmdDone                  → 等待擦除完成
 *   5. programMemory64WithECCGenerated → 每次写入 64-bit + 自动 ECC
 *   6. waitForCmdDone                  → 等待编程完成
 *   7. protectSector(addr)             → 重新锁定扇区
 *
 * 注意：
 *   - API 传入的是 Flash 地址，不是扇区编号
 *   - 擦写前关闭全局中断，防止 ISR 访问 Flash 导致 bus fault
 ******************************************************************/

#include "param_store.h"
#include "ti_msp_dl_config.h"
#include <string.h>

/* ==================== Flash 存储地址定义 ==================== */

/* 使用最后一个 Flash 扇区 (地址 0x0001FC00 ~ 0x00020000, 1KB) */
#define PARAM_FLASH_ADDR     0x0001FC00U

/* Magic 魔数：0x4D534B50 = ASCII "MSKP"，用于检验数据有效性 */
#define PARAM_MAGIC          0x4D534B50UL

/* 参数结构体: 8 个字段 + 保留, 共 24 字节 = 6 个 uint32_t */
typedef struct {
    uint32_t magic;          /* [31:0]   魔数校验 */
    int16_t  kp;             /* [47:32]  比例增益 */
    int16_t  kd;             /* [63:48]  微分增益 */
    int16_t  lost_gain;      /* [79:64]  丢线增益 */
    int16_t  base_speed;     /* [95:80]  基础速度 */
    int16_t  laps;           /* [111:96] 目标圈数 */
    int16_t  pivot_time;     /* [127:112] pivot最小保持时间 */
    uint8_t  mpu_mode;       /* [135:128] MPU显示模式 0=关 1=开 */
    uint8_t  pivot_speed;    /* [143:136] Pivot慢轮速度 0~200 */
    uint8_t  reserved[6];    /* [191:144] 保留 */
} ParamData_t;

/* ==================== 参数读取 ==================== */

/******************************************************************
 * 函 数 名 称：Param_Load
 * 函 数 说 明：从 Flash 固定地址读取保存的参数值
 * 函 数 形 参：输出指针（laps可传NULL表示跳过圈数恢复）
 * 函 数 返 回：true=成功读取并校验通过  false=未写过或数据异常
 * 备       注：Flash 可直接按地址读取，无需擦写流程
 ******************************************************************/
bool Param_Load(int16_t *kp, int16_t *kd, int16_t *lost_gain,
                int16_t *base_speed, int16_t *laps, int16_t *pivot_time,
                uint8_t *mpu_mode, int16_t *pivot_speed)
{
    const ParamData_t *p = (const ParamData_t *)PARAM_FLASH_ADDR;

    /* 魔数不匹配 → 从未保存过参数，或数据损坏 */
    if (p->magic != PARAM_MAGIC)
        return false;

    *kp         = p->kp;
    *kd         = p->kd;
    *lost_gain  = p->lost_gain;
    *base_speed = p->base_speed;
    if (laps != NULL)
        *laps = p->laps;
    if (pivot_time != NULL)
        *pivot_time = p->pivot_time;
    if (mpu_mode != NULL)
        *mpu_mode = p->mpu_mode;
    /* pivot_speed: 旧数据此字段为 0xFF（原 reserved），大于 200 时取默认值 */
    if (pivot_speed != NULL) {
        int16_t ps = (int16_t)p->pivot_speed;
        *pivot_speed = (ps >= 0 && ps <= 200) ? ps : 30;
    }

    /* 范围校验 */
    if (*kp         < 1   || *kp         > 100) return false;
    if (*kd         < 1   || *kd         > 100) return false;
    if (*lost_gain  < 1   || *lost_gain  > 100) return false;
    if (*base_speed < 50  || *base_speed > 500) return false;
    if (laps != NULL && (*laps < 1 || *laps > 9)) return false;

    return true;
}

/* ==================== 参数写入 ==================== */

/******************************************************************
 * 函 数 名 称：Param_Save
 * 函 数 说 明：擦除目标扇区后将参数写入 Flash
 * 函 数 形 参：五个待保存参数值
 * 函 数 返 回：true=写入成功  false=失败
 * 备       注：操作流程：清状态 → 关中断 → 解锁 → 擦除 → 编程 → 加锁 → 开中断
 *             全程约 1~3ms，K2 按键时调用无感知延迟
 ******************************************************************/
bool Param_Save(int16_t kp, int16_t kd, int16_t lost_gain,
                int16_t base_speed, int16_t laps, int16_t pivot_time,
                uint8_t mpu_mode, int16_t pivot_speed)
{
    ParamData_t data;
    data.magic       = PARAM_MAGIC;
    data.kp          = kp;
    data.kd          = kd;
    data.lost_gain   = lost_gain;
    data.base_speed  = base_speed;
    data.laps        = laps;
    data.pivot_time  = pivot_time;
    data.mpu_mode    = mpu_mode;
    data.pivot_speed = (uint8_t)((pivot_speed >= 0 && pivot_speed <= 200) ? pivot_speed : 30);
    memset(data.reserved, 0xFF, sizeof(data.reserved));

    DL_FLASHCTL_COMMAND_STATUS status;

    /* ---- 关闭全局中断（Flash 操作期间不可有 ISR 访问 Flash）---- */
    __disable_irq();

    /* 步骤 0：清除 Flash 状态寄存器 */
    DL_FlashCTL_executeClearStatus(FLASHCTL);

    /* 步骤 1：解锁目标扇区 */
    DL_FlashCTL_unprotectSector(FLASHCTL,
        PARAM_FLASH_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);

    /* 步骤 2：擦除扇区，使用 FromRAM 版本（单 bank Flash 必须从 RAM 执行）*/
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        PARAM_FLASH_ADDR, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        DL_FlashCTL_protectSector(FLASHCTL,
            PARAM_FLASH_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);
        __enable_irq();
        return false;
    }
    /* eraseMemoryFromRAM 内部阻塞至完成，仍需检查执行结果 */
    if (!DL_FlashCTL_waitForCmdDone(FLASHCTL)) {
        DL_FlashCTL_protectSector(FLASHCTL,
            PARAM_FLASH_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);
        __enable_irq();
        return false;
    }

    /* 步骤 3：一次性写入全部 16 字节（4 个 32-bit word = 2 个 Flash word）
     * 使用 Blocking FromRAM API → 内部自动：
     *   - 从 RAM 执行闪存命令（单 bank Flash 安全要求）
     *   - 解锁/加锁目标扇区
     *   - 硬件自动生成 ECC
     *   - 阻塞等待操作完成
     *
     * ★ 关键：一次调用写完 16 字节！
     * 旧代码分两次 programMemory64WithECCGenerated 调用，第一次完成后
     * Flash 控制器自动加锁所有扇区，导致第二次调用在已加锁的扇区上编程，
     * 后半段数据（lost_gain/base_speed/laps）写入失败，掉电后丢失。
     */
    status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, PARAM_FLASH_ADDR, (uint32_t *)&data, 6,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        __enable_irq();
        return false;
    }

    /* 步骤 4：恢复扇区写保护（Blocking API 内部已加锁，再调一次确保安全）*/
    DL_FlashCTL_protectSector(FLASHCTL,
        PARAM_FLASH_ADDR, DL_FLASHCTL_REGION_SELECT_MAIN);

    /* 恢复全局中断 */
    __enable_irq();

    /* 回读校验 */
    {
        const ParamData_t *verify = (const ParamData_t *)PARAM_FLASH_ADDR;
        if (verify->kp != kp || verify->kd != kd ||
            verify->lost_gain != lost_gain || verify->base_speed != base_speed ||
            verify->laps != laps || verify->pivot_time != pivot_time ||
            verify->mpu_mode != mpu_mode ||
            verify->pivot_speed != data.pivot_speed)
        {
            return false;
        }
    }

    return true;
}
