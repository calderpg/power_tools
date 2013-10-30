std_msgs/Header header
# Estimated information
duration TimeToEmpty
duration TimeToFull
# Information about output state
int32 NumOutputs
string[] OutputID
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
float64 RatedCapacity
float64 CurrentCapacity
float64 PercentCharge
float64 CurrentCharge
# Device options
uint8 Unknown=0
uint8 LinePower=1
uint8 Battery=2
uint8 UPS=3
uint8 Converter=10
# Type of power device
uint8 DeviceType
# Possible system states
uint8 Powered=0
uint8 Charging=1
uint8 Discharging=2
uint8 Empty=3
uint8 FullyCharged=4
uint8 PendingCharge=5
uint8 PendingDischarge=6
# State of the system
uint8 SystemState
# Number of cells
int32 NumCells
# Cell types (basic type)
uint8 NoType=0
uint8 LiIon=1
uint8 LiPo=2
uint8 LiFePO4=3
uint8 SLA=4
uint8 NiCd=5
uint8 NiMH=6
# Cell type
uint8 CellType
# Nominal values for the cells (all cells must be the same)
float64 NominalCellVoltage
float64 NominalCellCharge
# Actual values for each cell
float64[] CellVoltage
float64[] CellCharge
float64[] CellTemperature
# Information about the full power system
float64 SystemTemperature
string DeviceName
string DeviceID
string Manufacturer
