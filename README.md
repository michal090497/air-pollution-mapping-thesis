# C code for STM32 microcontroller unit
Microcontroller code based mainly on HAL library. Main purpose of this code was providing data from sensors, i.e. collecting data in proper moment, processing data and storing them in proper HTTP request.

<img alt="C" src="https://img.shields.io/badge/c%20-%2300599C.svg?&style=for-the-badge&logo=c&logoColor=white"/>
<img alt="C++" src="https://img.shields.io/badge/c++%20-%2300599C.svg?&style=for-the-badge&logo=c%2B%2B&ogoColor=white"/>

## Main parts of software

### Interruptions
Whole program uses timer interrupts as a base for every operation. Everything is handled with the use of flags.

### Communication with sensors
Digital sensors are connected via UART interface, so in functions there is code providing collecting and processing data.

### ADC usage
For analog sensor code provides ADC handling and proper calculations.

### Server connection
As the collected data should be sent on server, code responsible for getting data is merged with the code handling A9G module (GSM and GPS). The code for A9g module is provided by Wojciech Stróżecki.

### Hardware
This repository consists of code only. The hardware part is described in thesis.

## Credits
* Michał Kliczkowski - STM32 development
* Tomasz Jankowski - web application
* Wojciech Stróżecki - A9G development
* prof. dr hab. inż. Adam Dąbrowski - conceptual support and funding

## License
 
The MIT License (MIT)

Copyright (c) 2020 Michał Kliczkowski

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
