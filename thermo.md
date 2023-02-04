- uses mode 3 by default
- 8 bit command
- followed by 8 bit/ 16 bit/ 24 bit data
- read fromI ID register 0x05

## Methods

1. Id command - check if the device is connected and Spi is working

2. send method

```
fun send(command){}
```

3. getVoltage() - get current voltage from ADC in mV

```rust
fun getVoltage()->f64{}
```

4. convertToC() - converts the mvoltage to temperature C

```rust
fun convertToC(Voltage:f64)->f64{}
```

- will have 2 implementations
  - 1. using the lookup table
  - 2. using the equation

5. getTemp() - gets the data from the ADC and then converts it to temperature C

```rust
fun getTemp()->f64{}
```
