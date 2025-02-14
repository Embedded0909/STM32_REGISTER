# STM32 -- REG -- BASIC
Cách học: 
👌 Bước 01: Config toàn bộ lại thanh ghi 
👌 Bước 02: Cần dùng gì thì tìm hiểu cái đó

⚠️ Kiến thức là vô hạn vì vậy nếu thấy vấn đề sai sót hoặc không đúng lắm hãy liên hệ mình để sửa lại để tạo một cộng đồng clean code.
```
-----------------------------------
| CHƯƠNG    | NỘI DUNG            |
-----------------------------------
| CHƯƠNG 00 | GIỚI THIỆU          |
| CHƯƠNG 01 | RCC                 |
| CHƯƠNG 02 | GPIO                |
| CHƯƠNG 03 | AFIO                |
| CHƯƠNG 04 | EXTI                |
| CHƯƠNG 05 | ADC                 |
| CHƯƠNG 06 | TIMER               |
| CHƯƠNG 07 | PWM                 |
| CHƯƠNG 08 | UART                |
| CHƯƠNG 09 | I2C                 |
| CHƯƠNG 10 | SPI                 |
----------------------------------- 
```


## Chương 00: MỘT SỐ KIẾN THỨC CẦN HỌC KHI HỌC VỀ THANH GHI

Có rất nhiều kiến trúc ở thời điểm hiện tại
- ARM   STM     
- 8051  AT89S52 
- AVR   ATMEGA  
- PIC   PIC8 PIC32
- RISC  ESP32 (Dual core)
........ Rất nhiều kiến trúc .........

STM32 là core ARM Cortex-M3

### Set bit lên 1
Cái gì hoặc với 1 cũng bằng 1
```cpp
register |= (1 << 3);  // Đặt bit thứ 3 lên 1
```
### Clear bit
Cái gì và với 0 cũng bằng 0
```cpp
register &= ~(1 << 3);  // Đặt bit thứ 3 về 0
```
### Union
- Tiết kiệm bộ nhớ
- Dễ dàng quản lý
Ví dụ
```cpp
typedef union{
    uint8_t REG;
    struct{
        uint8_t BIT0: 1;
        uint8_t BIT1: 1;
        uint8_t BIT2: 1;
        uint8_t BIT3: 1;
        uint8_t BIT4: 1;
        uint8_t BIT5: 1;
        uint8_t BIT6: 1;
        uint8_t BIT7: 1;
    }BITS;
}__BIT8;

__BIT8 myData;

```
Lúc này thì myData sẽ là một đối tượng gồm có REG và BITS
Lúc này thì khi bạn đổi dữ liệu bất kỳ BIT nào thì biến REG cũng sẽ thay đổi bởi lẽ bản chất thì cả REG và BITS đều trỏ vào vị trí đầu tiên chính là BIO0 --> Đây là cơ chế union

### Define con trỏ chọn đến địa chỉ bất kì
Ví dụ vi điều khiển có GPIOA được đặt với địa chỉ là 0x12345678
GPIOA với kiểu dữ liệu là GPIO_Typedef mà mình muốn variable GPIOA của mình tự định nghĩa ra mapping được với địa chỉ kia thì làm như sau:
```cpp
#define GPIOA     ((volatile GPIO_TypeDef*) 0x12345678)
```
Khi bạn làm việc với phần cứng hoặc các tài nguyên mà giá trị của chúng có thể thay đổi ngoài tầm kiểm soát của chương trình (như thanh ghi của vi điều khiển,...) khi đó thì compiler có thể tối ưu hóa biến này đi

volatile ở đây là để compiler xác định nó là biến có thể bị thay đổi bất cứ lúc nào và không clear nó đi
## Chương 01: RCC
### 1.1 Cách config sysclock lên tối đa 72Mhz
![Alt text](image/RCC_01.png)

Các bước config lên 72 Mhz
```cpp
        // Bước 01: Enable HSE
	// Bước 02: Config Flash 
	// Bước 03: PLL x9	
	// Bước 04: Div clock
	// Bước 05: PLL as SysClk
```

### 1.2 Các thanh ghi để config sysclock lên 72Mhz
#### Thanh ghi CR
![alt text](image-1.png)

```cpp
bit 16: Cấu hình system hoạt động theo HSE
bit 17: Đợi cho HSE hoạt dộng 
bit 24: Cấu hình theo PLL
bit 25: Đợi cho PLL hoạt động 
```

#### Thanh ghi CFGR
![alt text](image/CFGR.png)

```cpp
PLLSRC: Chọn src cho PLL
PLLMUL[3:0]: Bộ nhân tần
PPRE2[2:0] : Bộ chia tần APB2
PPRE1[2:0] : Bộ chia tần APB1
HPRE[3:0]  : Bộ chia tần HPRE
SW: Chọn clock cho system
SWS: Chờ cho quá trình SW hoàn thành
```

⚠️⚠️⚠️ Lưu ý: Nếu bạn chỉ làm điều này thì sau khi làm xong thì system của bạn vẫn không thể lên được 72Mhz

Bản thân trong vi điều khiên có một khái niệm flash
Khi bạn chọn SYSCLK chạy ở 72 MHz, điều này ảnh hưởng đến tốc độ của nhiều thành phần trong hệ thống, bao gồm tốc độ truy xuất bộ nhớ Flash 
--> Chính vì vậy chúng ta cần config trong 1 thanh ghi nữa về FLASH (FLASH_ACR)
```cpp
Bits 2:0 LATENCY: Latency
000 Zero wait state, if 0 <= SYSCLK <= 24 MHz
001 One wait state, if 24 MHz < SYSCLK <= 48 MHz
010 Two wait states, if 48 MHz < SYSCLK <= 72 MHz

```
### 1.3 Bật clock của các ngoại vi

#### Clock APB2
Ví dụ các bạn cần bật Clock của GPIOA thì cần đến thanh ghi này
![alt text](image/APB2.png)
Đơn giản chỉ cần bật bit IOPA = 1 là được

## Chương 02: GPIO

😒 Mục tiêu:
+ Xác thực tính đúng đắn của RCC
+ Blink Led

GPIO bản chất là vào ra tín hiệu
Để blink led, đầu tiên chúng ta cần chọn ra một chân cắm vào led VD PA0 và thực hiện các lệnh sau
```cpp

Bước 01: Bật clk của PORTA (RCC bài trước)
Bước 02: Config Output, Input, PullUp, PullDown, PushPull, OpenDrain
Bước 03: Xuất tín hiệu điện áp mức HIGH, LOW để blink led

```

### 2.1 Define các thanh ghi
#### 2.1.1 Thanh ghi CRL

![alt text](image/CRL.png)

Thanh ghi CRL dùng để cấu hình IO, các Mode cho các Pin từ 0 đến 7

#### 2.1.2 Thanh ghi CRH

![alt text](image/CRH.png)

Thanh ghi CRH dùng để cấu hình IO, các Mode cho các Pin từ 8 đến 15

#### 2.1.3 Thanh ghi ODR

![alt text](image/ODR.png)

Thanh ghi dùng để xuất tín hiệu ra chân Pin

## Chương 03: AFIO

Alternate Functions - Nó cung cấp các chức năng thay thế cho các chân GPIO như
- UART, SPI, I2C, EXTI,...
### 3.1 


## I. Nested vectored interrupt controller (NVIC)

![Alt text](image/NVIC_01.png)

## II. EXTI registers
### 1. Interrupt mask register EXTI_IMR

Các line ngắt
![Alt text](image/NVIC_02.png)

Address offset: 0x00
Reset value: 0x0000 0000
![Alt text](image/NVIC_03.png)
Dùng để bật ngắt line, dùng line ngắt nào thì phải bật line ngắt đó lên
VD: Muốn dùng ngắt ngoài chân PA0 thì phải set bit thứ 0 lên 1
```cpp
MR0 = 1
```
### 2. Rising trigger selection register EXTI_RTSR
Dùng cho mode Rising
Address offset: 0x08
Reset value: 0x0000 0000

![alt text](image/image.png)
Dùng để bật mode Rising cho line bất kỳ
VD: Muốn dùng ngắt Rising cho line 0 thì phải set bit thứ 0 lên 1
```cpp
TR0 = 1
```
### 3. Falling trigger selection register EXTI_FTSR
Dùng cho mode Falling
Address offset: 0x0C
Reset value: 0x0000 0000
![alt text](image/image1.png)
Dùng để bật mode Falling cho line bất kỳ
VD: Muốn dùng ngắt Rising cho line 0 thì phải set bit thứ 0 lên 1
```cpp
TR0 = 1
```
### 4. Pending register (EXTI_PR)
Bản chất khi một chân ngắt được detect, bit tương ứng với chân đó sẽ được set lên 1 trong thanh ghi này
Vì vậy để các ngắt khác có thể được thực thi, thì sau khi thực hiện ngắt hiện tại xong, chúng ta cần clear
bit tại thanh ghi này bằng cách write 1
![alt text](image/image2.png)