#ifndef PARAMS_INIT_H
#define PARAMS_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

void loadini(void);
void InitCoreData(void);
void LoadParamsFromFlash(void);
void WriteDefaultParamsToFlash(void);
void InitSmartCalibration(void);

#ifdef __cplusplus
}
#endif
#endif
