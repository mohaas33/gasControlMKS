
# Gas monitoring system
This repository contain scripts for multi gas controller 647C in the sPHENIX TPC modules test stand.

## Running


### **Talk to the device**
Read the set-point of the 6-th flow controller:
`./talk_to_MKS.py FS 6 R`

### **Commands** 

- Read the set-point of the 6-th flow controller: 
`FS 6 R`
- Set the set-point of the 6-th flow controller to 1 liter per minute for Ar (146/1100*5*1.37): 
`FS 6 00146` 
- Enable the flow controller: 
`ON 6`
- Turn all flow controllers on:
`ON 0`
- Turn all flow controllers off:
`OF 0`
- Disable the flow controller: 
`OF 6`

### **Monitoring**

`./gasMonitor.py`