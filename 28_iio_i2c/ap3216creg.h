#ifndef AP3216C_H
#define AP3216C_H
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ap3216creg.h
版本	   	: V1.0
描述	   	: AP3216C寄存器地址描述头文件
***************************************************************/

#define AP3216C_ADDR    	0X1E	/* AP3216C器件地址  */

/* AP3316C寄存器 */
#define AP3216C_SYSTEMCONG	0x00	/* 配置寄存器       */
#define AP3216C_INTSTATUS	0X01	/* 中断状态寄存器   */
#define AP3216C_INTCLEAR	0X02	/* 中断清除寄存器   */
#define AP3216C_IRDATALOW	0x0A	/* IR数据低字节     */
#define AP3216C_IRDATAHIGH	0x0B	/* IR数据高字节     */
#define AP3216C_ALSDATALOW	0x0C	/* ALS数据低字节    */
#define AP3216C_ALSDATAHIGH	0X0D	/* ALS数据高字节    */
#define AP3216C_PSDATALOW	0X0E	/* PS数据低字节     */
#define AP3216C_PSDATAHIGH	0X0F	/* PS数据高字节     */

#define AP3216C_ALSCONFIG	0X10	/* ALS配置寄存器 */
#define AP3216C_PSCONFIG	0X20	/* PS配置寄存器 */
#define AP3216C_PSLEDCONFIG 0X21	/* LED配置寄存器 */

#endif
