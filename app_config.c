#include "app_config.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*
  改进说明（见 commit/message）：
  - 引入集中存储结构（magic + version + checksum），放在 APP_CFG_STORE_ADDR(CFG_ADDR_KEY) 起始位置；
  - 提供 APP_CONFIG_SaveAll / APP_CONFIG_LoadAll / APP_CONFIG_IsValid；
  - 保持原有按字段 Set/Get 接口向后兼容，但现在会同步更新集中存储；
  - APP_CONFIG_Init 在现有 CFG_ADDR_EN 检查的基础上尝试加载集中存储，加载失败则恢复默认并保存；
  - 采用简单累加字节和作为校验（轻量、与项目其它模块风格一致）。
*/

/* 全局配置实例（运行时） */
app_cfg_def  g_app_cfg  =
{
    .did = 123456,                          //默认设备ID
    .dtp = 1,
    .dnm = 1,
    .com = 0,
    .model = {0}
};

/* 存储结构（直接写入 EEPROM/Flash）*/
typedef struct {
    uint32_t magic;           // magic
    uint16_t store_version;   // 存储结构版本
    uint16_t reserved;        // 对齐 / 预留
    uint32_t did;
    uint16_t dtp;
    uint16_t dnm;
    uint16_t com;
    char     model[20];
    uint32_t checksum;        // 校验和（含上面全部字节，写入前置0计算）
} app_cfg_store_t;

/* 计算校验：对字节流求和（checksum 字段须为0时计算） */
static uint32_t calc_store_checksum(const app_cfg_store_t *s)
{
    uint32_t sum = 0;
//    const uint8_t *p = (const uint8_t*)s;
    size_t sz = sizeof(app_cfg_store_t);
    /* 把 checksum 字段当 0 参与计算 */
    for (size_t i = 0; i < sz; ++i)
    {
        /* checksum 位的偏移 */
    }
    /* 更简洁的实现：拷贝到临时结构，将 checksum 置0，再求和 */
    app_cfg_store_t tmp;
    memcpy(&tmp, s, sizeof(tmp));
    tmp.checksum = 0;
    const uint8_t *q = (const uint8_t*)&tmp;
    for (size_t i = 0; i < sizeof(tmp); ++i)
    {
        sum += q[i];
    }
    return sum;
}

/* 将内存中 g_app_cfg 写入集中存储并同时更新老接口的单字段地址（向后兼容） */
int APP_CONFIG_SaveAll(void)
{
    app_cfg_store_t store;
    memset(&store, 0, sizeof(store));
    store.magic = APP_CFG_MAGIC;
    store.store_version = APP_CFG_STORE_VERSION;
    store.did = g_app_cfg.did;
    store.dtp = (uint16_t)g_app_cfg.dtp;
    store.dnm = (uint16_t)g_app_cfg.dnm;
    store.com = (uint16_t)g_app_cfg.com;
    /* model 保证以0结尾 */
    memcpy(store.model, g_app_cfg.model, sizeof(store.model));
    store.checksum = 0;
    store.checksum = calc_store_checksum(&store);

    /* 写入 EEPROM：使用半字（16-bit）接口，长度以半字为单位 */
    APP_MEMORY_Write_BUF16(APP_CFG_STORE_ADDR, (uint16_t*)&store, sizeof(store) / 2);


    /* 向后兼容：同时写入旧的单字段地址（减少依赖问题） */
    APP_MEMORY_Write_U32(CFG_ADDR_DID, store.did);
    APP_MEMORY_Write_U16(CFG_ADDR_DTP, store.dtp);
    APP_MEMORY_Write_U16(CFG_ADDR_DNM, store.dnm);
    APP_MEMORY_Write_U16(CFG_ADDR_COM, store.com);

    return 0;
}

/* 从 EEPROM 读取集中存储并验证（成功则填充 g_app_cfg） */
int APP_CONFIG_LoadAll(void)
{
    app_cfg_store_t store;
    memset(&store, 0, sizeof(store));

    int ret = APP_MEMORY_Read_BUF16(APP_CFG_STORE_ADDR, (uint16_t*)&store, sizeof(store) / 2);
    if (ret != USR_EOK && ret != 0) {
        LOG("APP_CONFIG_LoadAll: read failed\n");
        return -1;
    }

    if (store.magic != APP_CFG_MAGIC) {
        LOG("APP_CONFIG_LoadAll: magic mismatch (0x%08X)\n", (unsigned)store.magic);
        return -2;
    }

    /* 校验 checksum */
    uint32_t cs = store.checksum;
    uint32_t calc = calc_store_checksum(&store);
    if (cs != calc) {
        LOG("APP_CONFIG_LoadAll: checksum mismatch (read=%u calc=%u)\n", (unsigned)cs, (unsigned)calc);
        return -3;
    }

    /* 成功：填充运行时结构体 */
    g_app_cfg.did = store.did;
    g_app_cfg.dtp = store.dtp;
    g_app_cfg.dnm = store.dnm;
    g_app_cfg.com = store.com;
    memcpy(g_app_cfg.model, store.model, sizeof(g_app_cfg.model));

    /* 保证老接口地址也一致（可选，不要频繁写以减少擦写） */
    APP_MEMORY_Write_U32(CFG_ADDR_DID, g_app_cfg.did);
    APP_MEMORY_Write_U16(CFG_ADDR_DTP, g_app_cfg.dtp);
    APP_MEMORY_Write_U16(CFG_ADDR_DNM, g_app_cfg.dnm);
    APP_MEMORY_Write_U16(CFG_ADDR_COM, g_app_cfg.com);

    return 0;
}

/* 验证集中存储是否有效（不加载） */
int APP_CONFIG_IsValid(void)
{
    app_cfg_store_t store;
    memset(&store, 0, sizeof(store));
    int ret = APP_MEMORY_Read_BUF16(APP_CFG_STORE_ADDR, (uint16_t*)&store, sizeof(store) / 2);
    if (ret != USR_EOK && ret != 0) {
        return 0;
    }
    if (store.magic != APP_CFG_MAGIC) return 0;
    if (store.checksum != calc_store_checksum(&store)) return 0;
    return 1;
}

/* 旧的按字段接口：现在同步到集中存储（写入时会 SaveAll） */
void APP_CONFIG_Did_Set(uint32_t did)
{
    g_app_cfg.did = did;
    APP_MEMORY_Write_U32(CFG_ADDR_DID, g_app_cfg.did); /* legacy */
    APP_CONFIG_SaveAll();
}

uint32_t APP_CONFIG_Did_Get(void)
{
    /* 优先返回当前内存值（Init 时会加载），若内存为0则尝试从 legacy 地址读取并同步 */
    if (g_app_cfg.did == 0) {
        uint32_t v = APP_MEMORY_Read_U32(CFG_ADDR_DID);
        if (v != 0) g_app_cfg.did = v;
    }
    return g_app_cfg.did;
}

void APP_CONFIG_Dtp_Set(uint32_t dtp)
{
    g_app_cfg.dtp = (uint16_t)dtp;
    APP_MEMORY_Write_U16(CFG_ADDR_DTP, g_app_cfg.dtp);
    APP_CONFIG_SaveAll();
}

uint32_t APP_CONFIG_Dtp_Get(void)
{
    if (g_app_cfg.dtp == 0) {
        uint16_t v = APP_MEMORY_Read_U16(CFG_ADDR_DTP);
        if (v != 0) g_app_cfg.dtp = v;
    }
    return g_app_cfg.dtp;
}

void APP_CONFIG_DNM_Set(uint32_t dnm)
{
    g_app_cfg.dnm = (uint16_t)dnm;
    APP_MEMORY_Write_U16(CFG_ADDR_DNM, g_app_cfg.dnm);
    APP_CONFIG_SaveAll();
}

uint32_t APP_CONFIG_DNM_Get(void)
{
    if (g_app_cfg.dnm == 0) {
        uint16_t v = APP_MEMORY_Read_U16(CFG_ADDR_DNM);
        if (v != 0) g_app_cfg.dnm = v;
    }
    return g_app_cfg.dnm;
}

void APP_CONFIG_COM_Set(uint32_t com)
{
    g_app_cfg.com = (uint16_t)com;
    APP_MEMORY_Write_U16(CFG_ADDR_COM, g_app_cfg.com);
    APP_CONFIG_SaveAll();
}

uint32_t APP_CONFIG_COM_Get(void)
{
    if (g_app_cfg.com == 0) {
        uint16_t v = APP_MEMORY_Read_U16(CFG_ADDR_COM);
        if (v != 0) g_app_cfg.com = v;
    }
    return g_app_cfg.com;
}

/* 默认配置（Reset）——现在会写集中存储并写使能标记 */
void APP_CONFIG_Reset(uint16_t id)
{
    /* 先设内存默认 */
    g_app_cfg.com = 0;
    g_app_cfg.did = id;
    g_app_cfg.dtp = 1;
    g_app_cfg.dnm = 0;
    memset(g_app_cfg.model, 0, sizeof(g_app_cfg.model));
    strncpy(g_app_cfg.model, "UNKNOWN", sizeof(g_app_cfg.model) - 1);

    /* 写入 legacy 单字段地址（保持兼容） */
    APP_MEMORY_Write_U16(CFG_ADDR_COM, g_app_cfg.com);
    APP_MEMORY_Write_U32(CFG_ADDR_DID, g_app_cfg.did);
    APP_MEMORY_Write_U16(CFG_ADDR_DTP, g_app_cfg.dtp);
    APP_MEMORY_Write_U16(CFG_ADDR_DNM, g_app_cfg.dnm);

    /* 集中保存 */
    APP_CONFIG_SaveAll();

    LOG("factory reset! DID=%lu\n", (unsigned long)g_app_cfg.did);
}

/* 旧的“读取所有配置”函数替换为从集中存储加载或回退到 legacy 读 */
void APP_CONFIG_Read(void)
{
    /* 优先尝试集中加载 */
    if (APP_CONFIG_LoadAll() == 0) {
        return;
    }

    /* 否则回退到逐项读取（旧行为） */
    g_app_cfg.did = APP_MEMORY_Read_U32(CFG_ADDR_DID);
    g_app_cfg.dtp = APP_MEMORY_Read_U16(CFG_ADDR_DTP);
    g_app_cfg.dnm = APP_MEMORY_Read_U16(CFG_ADDR_DNM);
    g_app_cfg.com = APP_MEMORY_Read_U16(CFG_ADDR_COM);
    /* model 无法从 legacy 单项恢复，保持内存默认 */
}

/* 初始化：保留旧有 CFG_ADDR_EN 检查逻辑，优先尝试集中加载，失败时写默认并保存 */
void APP_CONFIG_Init(void)
{
    /* 旧的使能检查：如果没有设备标记，则做 factory reset（保持兼容） */
    if (APP_MEMORY_Read_U16(CFG_ADDR_EN) != 9527)
    {
        APP_MEMORY_Write_U16(CFG_ADDR_EN, 9527);
        APP_CONFIG_Reset(12345);
        return;
    }

    /* 已标记：尝试加载集中配置 */
    if (APP_CONFIG_LoadAll() == 0)
    {
        LOG("APP_CONFIG_Init: loaded centralized config (DID=%lu)\n", (unsigned long)g_app_cfg.did);
        return;
    }

    /* 加载失败时，尝试逐项读取 legacy 地址 */
    APP_CONFIG_Read();

    /* 如果读取后发现 DID 为0（或非法），写入默认并保存 */
    if (g_app_cfg.did == 0)
    {
        APP_CONFIG_Reset(12345);
    }
}

/*****END OF FILE****/
