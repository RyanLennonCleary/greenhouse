# greenhouse
A project to optimize growing conditions and automate care for a mango tree in a small greenhouse

TASK

interface with dht11 temperature sensor and decode temperature readings correctly

outcome - reading from sensor. sensor uses monobus protcol requiring gpio pin to switch from input to output, which currently uses too many cycles through the API

TODO
reduce number of cycles required to read from sensor so that other applications can run
  - determine number of cycles currently being used 
  - set a target number
