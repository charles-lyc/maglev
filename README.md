# maglev

## introduce
This is just another magnetic leviation.  
The controling parameter should be tunned for better stabilizing.  

![pcb_1](/img/pcb_1.jpg)
![pcb_2](/img/pcb_2.jpg)
</br></br>

## reference
这里需要求出速度，因为普通的微分会有很大噪音，普通lpf也会有较大的滞后，所以寻求一些别的求导技巧。  
例如5点法（求出的是上上个值的倒数），或者追求更低的延时使用拉格朗日插值，
![5 point diff](/img/5_point_diff.jpg)


To be continue...