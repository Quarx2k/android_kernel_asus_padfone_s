#ifndef ___ICATCH7002A_H__
#define ___ICATCH7002A_H__

#include "msm_sensor.h"
#include <media/asus_cam_intf_type.h>
//#include "yuv_sensor.h"

#define WITH_INT // no define: polling mode, define: interrupt mode
#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

enum pmic_flash_status_t {
  	PMIC_FLASH_STATUS_FLASH_ON,
	PMIC_FLASH_STATUS_FLASH_OFF
};

#ifdef WITH_INT
irqreturn_t iCatch_irq_handler(int irq, void *data);
#endif

int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val);
int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val);
int icatch_i2c_debuginit(void);
int sensor_set_mode_main_camera(int  res, bool bSpeedUpPreview);
unsigned int get_fw_version_in_bin(char* binfile_path);
int sensor_set_mode_second_camera(int res);
unsigned int get_fw_version_in_isp_fromISP(void);
unsigned int get_fw_version_in_isp(void);
void wait_for_next_frame(void);

void create_iCatch_proc_file(void);
void remove_iCatch_proc_file(void);
void iCatch_init(void);
void iCatch_deinit(void);
void iCatch_release_sensor(void);
void wait_for_AWB_ready(void);
void iCatch_enable_exif(bool enable, bool is_preExif);
void iCatch_set_ev_mode(int16_t mode);                      //ASUS_BSP Stimber  "Implement ISP settings - EV mode"
void create_iCatch_switch_file(void);
void remove_iCatch_switch_file(void);
    
// ASUS_BSP +++ jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"
void iCatch_set_iso_mode(int16_t mode);                     //ASUS_BSP jim3_lin "Implement ISP settings - ISO mode"
void iCatch_set_wb_mode(int16_t mode);                      //ASUS_BSP jim3_lin "Implement ISP settings - WB mode"
void iCatch_set_flicker_mode(int16_t mode);                 //ASUS_BSP jim3_lin "Implement ISP settings - Flicker mode"
void iCatch_set_aeclock_mode(int16_t mode);                 //ASUS_BSP jim3_lin "Implement ISP settings - AEC-Lock mode"
void iCatch_set_awblock_mode(int16_t mode);                 //ASUS_BSP jim3_lin "Implement ISP settings - AWB-Lock mode"
void iCatch_set_led_mode(int16_t mode);                     //ASUS_BSP jim3_lin "Implement ISP settings - LED mode"
// ASUS_BSP --- jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement Scene/Effect mode for ISP"
void iCatch_set_scene_mode(int16_t mode);                   //ASUS_BSP jim3_lin "Implement ISP settings - Scene mode"
void iCatch_set_effect_mode(int16_t mode);                  //ASUS_BSP jim3_lin "Implement ISP settings - Effect mode"
// ASUS_BSP --- jim3_lin "Implement Scene/Effect mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"
void iCatch_set_wdr_mode(int16_t mode);                     //ASUS_BSP jim3_lin "Implement ISP settings - WDR mode"
void iCatch_set_aura_value(int16_t mode);                   //ASUS_BSP jim3_lin "Implement ISP settings - Aura mode"
// ASUS_BSP --- jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"

void iCatch_set_ultrapixel(int16_t mode);                   //ASUS_BSP jim3_lin "Implement DIT postprocess"
void iCatch_set_wb_effect_scene(uint16_t mode);              //ASUS_BSP jim3_lin "Fix Scene/Effect/WB setting order"
void iCatch_set_af_mode(int16_t mode);
    
void iCatch_start_AF(bool on, cam_focus_mode_type mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w);
uint16_t iCatch_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl);
void iCatch_get_exif(struct exif_cfg *exif);	//ASUS_BSP Stimber "Implement EXIF info for camera with ISP"

void iCatch_set_hal_general_cmd(struct kernel_mct_event_control_parm_t *hal_cmd);	//ASUS_BSP Stimber "implement hal general set command"
void iCatch_enable_autonight(bool enable);                   //ASUS_BSP jim3_lin "Implement DIT postprocess-Mode2"
void iCatch_set_gyro_mode(uint8_t mode);					//ASUS_BSP Stimber "Implement gyro detect mode"
void iCatch_set_max_fps_mode(uint8_t fps);					//ASUS_BSP Stimber "Implement max frame rate mode"

void enable_isp_interrupt(void);
void disable_isp_interrupt(void);
void enable_isp_torch_interrupt(void);
void disable_isp_torch_interrupt(void);

void iCatch_create_workqueue(void);
void  iCatch_destroy_workqueue(void);

void setFixFPS(int settingVal);
void setMiniISO(int settingVal);
void setCaptureVideoMode(int settingVal);

int32_t led_pmic_flash_enable(int enable, int hw_strobe);
int32_t led_pmic_torch_enable(int enable, int hw_strobe);
#endif //___ICATCH7002A_H__
