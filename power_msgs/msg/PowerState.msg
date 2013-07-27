std_msgs/Header header
# Estimated information
duration TimeToEmpty
duration TimeToFull
# Information about output state
int32 NumOutputs
string[] OutputId
float64[] OutputPower
float64[] OutputVoltage
float64[] NominalOutputVoltage
# Information about input state
int32 NumInputs
string[] InputID
float64[] InputPower
float64[] InputVoltage
float64[] NominalInputVoltage
# Hardware information
float64 RatedMaxCharge
float64 CurrentMaxCharge
float64 PercentCharge
float64 CurrentCharge
int32 NumCells
uint8 None=0
uint8 SLA=1
uint8 NiCd=2
uint8 NiMH=3
uint8 LiIon=4
uint8 LiPo=5
uint8 CellType
float64 NominalCellVoltage
float64 NominalCellCharge
float64[] CellVoltage
float64[] CellCharge
float64[] CellTemperature
float64 SystemTemperature
string DeviceName
string DeviceID
string Manufacturer
