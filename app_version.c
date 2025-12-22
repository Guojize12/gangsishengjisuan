#include "app_config.h"
#include "app_version.h"

static const uint8_t g_version_soft[3] = {1, 0, 3}; //软件版本
static const uint8_t g_version_hard[3] = {3, 0, 1}; //硬件版本

char g_version_model[20] = "GSS-01A-001";



#define TY_LOGO    "TY STC SLAVE"
#define TY_LINE1   "************************************************"
#define TY_LINE2   "**"

void APP_VERSION_Print_Logo(void)
{
    int len1, len2, len3;
    len1 = strlen(TY_LOGO);
    len2 = strlen(TY_LINE1);
    len3 = strlen(TY_LINE2);
    int num = (len2 - len1 - len3 * 2) / 2;

    LOG("\n");
    LOG(TY_LINE1);
    LOG("\n");
    LOG(TY_LINE2);
    for (int i = 0; i < num; i++)
    {
        LOG(" ");
    }
    LOG(TY_LOGO);
    if (len1 % 2 > 0)
    {
        num++;
    }
    for (int i = 0; i < num; i++)
    {
        LOG(" ");
    }
    LOG(TY_LINE2);
    LOG("\n");
    LOG(TY_LINE1);
    LOG("\n");
}

void APP_VERSION_Print(void)
{
    APP_VERSION_Print_Logo();
    LOG("Version soft: %d.%02d.%02d\n", g_version_soft[0], g_version_soft[1], g_version_soft[2]);
    LOG("Version hard: %d.%02d.%02d\n", g_version_hard[0], g_version_hard[1], g_version_hard[2]);
    LOG("Device ID: %d\n", g_app_cfg.did);
    LOG("Device Num: %d\n", g_app_cfg.dnm);
    LOG("Device TYPE: %d\n", g_app_cfg.dtp);
}

uint32_t APP_VERSION_Get_Soft_U32(void)
{
    uint32_t version = g_version_soft[0] * 10000 + g_version_soft[1] * 100 + g_version_soft[2];
    return version;
}
uint32_t APP_VERSION_Get_Hard_U32(void)
{
    uint32_t version = g_version_hard[0] * 10000 + g_version_hard[1] * 100 + g_version_hard[2];
    return version;
}
void APP_VERSION_Get_Soft(uint8_t *version_soft)
{
    version_soft[0] = g_version_soft[0];
    version_soft[1] = g_version_soft[1];
    version_soft[2] = g_version_soft[2];
}

void APP_VERSION_Get_Hard(uint8_t *version_hard)
{
    version_hard[0] = g_version_hard[0];
    version_hard[1] = g_version_hard[1];
    version_hard[2] = g_version_hard[2];
}

char *APP_VERSION_Get_Model(void)
{
    return g_version_model;
}


/*****END OF FILE****/

