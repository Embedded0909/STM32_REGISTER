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