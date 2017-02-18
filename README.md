* schedule: http://wsn-testbed.it.uu.se:5000/
* real-time Grafana: http://wsn-testbed.it.uu.se:3000/dashboard/db/experiment-overview?from=now-1m&to=now

## most significant runs

ours

| run  | duration | energy         | reliability   | median latency |
|------|----------|----------------|---------------|---------------:|
|  605 |     300  | 722.801148071  |         19/70 |  6366.0        |
|  558 |     300  | 720.823209152  |         34/70 |  5888.5        |

competition

| run  | duration | energy         | reliability   | median latency | team |
|------|----------|----------------|---------------|----------------|------|
|  548 |     150  | 356.910110386  |         34/34 | 16090.5        |    1 |

## setup

local

| EUI64                     |GoLogic | role         | Tengfei | Thomas |
|---------------------------|--------|--------------|---------|--------|
| `14-15-92-00-12-e6-3b-dd` | A8     | sensing node | `COM20` | `COM3` |
| `14-15-92-00-12-e6-43-0f` |        | hop 1, A     | `COM16` | `COM4` |
| `14-15-92-00-12-e6-6f-16` | B1     | hop 1, B     | `COM19` | `COM5` |
| `14-15-92-00-12-e6-bb-5e` |        | hop 2, A     | `COM5`  | `COM6` |
| `14-15-92-00-12-e6-b9-57` | B8     | hop 2, B     | `COM21` | `COM7` |
| `14-15-92-00-12-e6-79-05` |        | sink node    | `COM18` | `COM8` |

testbed

| EUI64                     | log file      | rank | role         |
|---------------------------|---------------|------|--------------|
| `14-15-92-00-16-bf-86-a0` | `log_116.txt` |      | sensing node |
| `14-15-92-00-13-b7-6e-1f` | `log_132.txt` |      |              |
| `14-15-92-00-16-c0-94-61` | `log_136.txt` |      |              |
| `14-15-92-00-16-c0-83-c9` | `log_139.txt` |      |              |
| `14-15-92-00-16-c0-51-43` | `log_142.txt` |      |              |
| `14-15-92-00-10-cf-c0-9e` | `log_143.txt` |      |              |
| `14-15-92-00-16-c0-57-88` | `log_144.txt` |      |              |
| `14-15-92-00-16-c0-6c-1b` | `log_145.txt` |      |              |
| `14-15-92-00-16-c0-7a-3c` | `log_146.txt` |      |              |
| `14-15-92-00-16-c0-68-29` | `log_147.txt` |      |              |
| `14-15-92-00-16-c0-59-9d` | `log_149.txt` |      |              |
| `14-15-92-00-10-58-20-b2` | `log_151.txt` |      |              |
| `14-15-92-00-16-c0-5a-de` | `log_153.txt` |      |              |
| `14-15-92-00-16-c0-54-60` | `log_154.txt` |      |              |
| `14-15-92-00-16-bf-94-44` | `log_157.txt` |      |              |
| `14-15-92-00-16-c0-58-a5` | `log_158.txt` |      |              |
| `14-15-92-00-0d-25-b6-8b` | `log_159.txt` |      |              |
| `14-15-92-00-13-b7-7a-ce` | `log_200.txt` |      |              |
| `14-15-92-00-13-b7-70-01` | `log_205.txt` |      |              |
| `14-15-92-00-13-ca-e4-ab` | `log_219.txt` |      | sink node    |

## light sensor calibration

* light OFF: sensor value ~80
* light ON: sensor value ~625
* recommended threshold: 400

## `#define`

* `LOCAL_SETUP`
    * different short addresses for different motes
    * change forwarding rule to force multi-hop
* `LIGHTPIN_ALLMOTES`
    * toggle `P2.3` on all motes, not just sink mote
* `UART_HOP`
    * when retransmitting a DATA packet, the mote prints the hop count it will send (i.e. it's own)
* `ENABLE_LEDS`
    * blink LEDs on all motes
* `ENABLE_DEBUGPINS`
    * toggle debugpins on all motes    

| `flashflood_local`  | `flashflood_testbed_debug` | `flashflood_testbed_release` |
|---------------------|----------------------------|------------------------------|
| `LOCAL_SETUP`       |                            |                              |
| `LIGHTPIN_ALLMOTES` | `LIGHTPIN_ALLMOTES`        |                              |
| `UART_HOP`          | `UART_HOP`                 |                              |
| `ENABLE_LEDS`       |                            |                              |
| `ENABLE_DEBUGPINS`  |                            |                              |

## DSN contents and use

```
  7 6 5 4 3 2 1 0
 +-+-+-+-+-+-+-+-+
 |L| hop |  seq  |
 +-+-+-+-+-+-+-+-+
```

* `L`: state light (0=off, 1=high)
* `seq`: increments at each new light switch
* `hop`: sensing node is hop 0

```
local setup
                my_addr=2     my_addr=3     my_addr=4
                 my_hop=1      my_hop=2      my_hop=3
           DATA     1A------------2A------------3A        ACK
          dest=2   /      ACK           DATA      \     dest=null
           seq=1  /     dest=null     dest=4       \     seq=1
           hop=2 /       seq=1         seq=1        \    hop=4 
 my_addr=1      /        hop=2         hop=4         \        my_addr=5
  my_hop=0     /                                      \        my_hop=4  
          sensing                                     sink

----------------------------------------------------------------------------

  my_addr=0x11                                          my_addr=0x11
          sensing                                     sink
               \                                      /
         DATA   \         ACK            DATA        /    ACK
       dest=0x11 \      dest=null      dest=0x11    /   dest=null
        seq=1     \      seq=1          seq=1      /     seq=1
        hop=0      \     hop=0          hop=2     /      hop=2
                    1B------------2B------------3B
                my_addr=0x11  my_addr=0x11  my_addr=0x11

testbed setup
```

## packet format

DATA

```
    0     1     2     3     4     5     6     7     8
 +-----+-----+-----+-----+-----+-----+-----+-----+-----+
 |    FCF    | DSN |   PANID   |   dest    |    CRC    |
 +-----+-----+-----+-----+-----+-----+-----+-----+-----+
```

ACK

```
    0     1     2     3     4
 +-----+-----+-----+-----+-----+
 |    FCF    | DSN |    CRC    |
 +-----+-----+-----+-----+-----+
```

## reference power consumption

| Project             | explanation         | raw results                                                                                                        | average current | energy 1min run | energy 5min run |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------- | ---------------:|----------------:|
| `ref_radiorx`       | radio on rx         | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256830000&to=1487256900000) |        23-24 mA | 146.27 J        | 731.35 J        |
| `ref_radioosc`      | radio oscillator on | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256945000&to=1487257020000) |        23-24 mA | 146.25 J        | 731.25 J        |
| `ref_radiooff_lpm0` | radio off, LPM0     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256340000&to=1487256410000) |         1.90 mA |  17.32 J        |  86.60 J        |
| `ref_radiooff_lpm3` | radio off, LPM3     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256510000&to=1487256590000) |         1.80 mA |  16.16 J        |  80.80 J        |
| `ref_radiooff_lpm4` | radio off, LPM4     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256650000&to=1487256730000) |         1.75 mA |  16.29 J        |  81.45 J        |

## Timer A

* 32 kHz (ACLK), continuous mode
* never stops, rolls over every ~2s

| interrupt      | description |
|----------------|-------------|
| `CCR1` compare | Used to calibrate ~5 MHz clock speed: how many subticks in 7 ticks? Fires every 32*7 ticks (6836us), compare TBR against last value of TBR. Then `value>>5` is the number of subticks in 7 ticks. |
| `CCR2` compare | Used to trigger light sensor sampling. Fires every 100 ticks (~3050us) |
| `overflow`     | _enabled by default, but no action_ |

## Timer B

* 5 MHz (SMCLK), continuous mode
* never stops (TODO: needs fixing)

| interrupt        | description |
|------------------|-------------|
| `CCR1` capture   | Triggered by the SFD pin toggling, triggers an interrupt. Do nothing on rising edge (start of frame). On falling edge (end of frame), read packet and schedule data transmission, if appropriate. |
| `CCR2` compare   | Fires when it's time to forward the packet, around 213.5us after end of frame of the ACK |
| `overflow`       | _enabled by default, but no action_ |

## pins

| pin  | name           | role      | description |
|------|----------------|-----------|-------------|
| P2.3 | `light`        | light pin | output pin on the sink node (possibly on all nodes) |
| P3.4 | `timerAisr`    | debugpin  | high during Timer A ISR |
| P6.6 | `timerBisr`    | debugpin  | high during Timer B ISR |
| P2.6 | `calibration`  | debugpin  | toggle at beginning of calibration |
| P3.5 | `sfd`          | debugpin  | reflects status of SFD |
| P6.7 | `radio`        | debugpin  | radio's oscillator on |

## leds

| led   | pin  | name    |
|-------|------|---------|
| red   | P5.4 |         |
| green | P5.5 |         |
| blue  | P5.6 | `light` |
