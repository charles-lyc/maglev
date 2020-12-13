# maglev

## introduce
This is just another magnetic leviation.  

![pcb_1](/doc/img/pcb_1.jpg)
![pcb_2](/doc/img/pcb_2.jpg)
</br></br>

## reference
磁铁的阻尼几乎为零，所以这里需要求出速度，利用速度环增加阻尼。
普通的微分会有很大噪音，普通lpf也会有较大的滞后，所以寻求一些别的求导技巧。例如5点法（估计出上上个采样点的倒数），或者使用拉格朗日插值。  
![5 point diff](/doc/img/5_point_diff_1.jpg)


To be continue...
