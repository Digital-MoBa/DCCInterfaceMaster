#ifndef __DCCHARDWARE_H__
#define __DCCHARDWARE_H__


#ifdef __cplusplus
extern "C"
{
#endif

void setup_DCC_waveform_generator(void);
void DCC_waveform_generation_hasshin(void);
void DCC_stop_output_signal(void);
void DCC_run_output_signal(void);

#ifdef __cplusplus
}
#endif

#endif //__DCCHARDWARE_H__