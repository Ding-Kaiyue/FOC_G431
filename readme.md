* 可尝试采用不定频控制，高频任务在while{1}里执行，频率理论上是170M，但需除去中断的时间

  与定频控制频率对比测试方法：led闪烁，测量led两端电压，电压高则工作频率高，电压低则工作频率低

* ADC开启连续转换模式，在初始化开启

![img](https://img-blog.csdnimg.cn/82d56f5a66cf4516a493cd978b7ff4b9.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA6LS-c2Fpc2Fp,size_20,color_FFFFFF,t_70,g_se,x_16)

![img](https://img-blog.csdnimg.cn/d155ee2428934cb1bea5d390e97c6955.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBA6LS-c2Fpc2Fp,size_20,color_FFFFFF,t_70,g_se,x_16)

* 可能导致的异常：ADC转化周期为2.5cycles，开启ADC转化HAL_ADC_Start_DMA(&hadc1)后，程序运行异常，修改ADC转化周期为640.5cycles(不影响，都很快)170M/4/（640.5+12.5）= 65084次中断
* 以上两条可以用一个us级的定时器实现，就像现在这样的TIM6 50us触发一次采样，如果FOC运行有问题，可以提高TIM6频率