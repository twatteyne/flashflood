* schedule: http://wsn-testbed.it.uu.se:5000/
* real-time Grafana: http://wsn-testbed.it.uu.se:3000/dashboard/db/experiment-overview?from=now-1m&to=now

## setup

local

TODO

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

## `#define`

* `LOCAL_SETUP`
    * enable debug pins on all motes
    * change forwarding rule to force multi-hop
    * different short addresses for different motes
* `LIGHTPIN_ALLMOTES`
    * toggle `P2.3` on all motes, not just sink mote
* `UART_HOP`
    * when retransmitting a DATA packet, the mote prints the hop count it will send (i.e. it's own)

## DSN contents

```
  7 6 5 4 3 2 1 0
 +-+-+-+-+-+-+-+-+
 |L| hop |  seq  |
 +-+-+-+-+-+-+-+-+
```

* `L`: state light (0=off, 1=high)
* `seq`: increments at each new light switch
* `hop`: sensing node is hop 0

## reference power consumption

| Project             | explanation         | raw results                                                                                                        | average current |
| ------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------- |
| `ref_radiorx`       | radio on rx         | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256830000&to=1487256900000) |         23-24mA |
| `ref_radioosc`      | radio oscillator on | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256945000&to=1487257020000) |         23-24mA |
| `ref_radiooff_lpm0` | radio off, LPM0     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256340000&to=1487256410000) |          1.90mA |
| `ref_radiooff_lpm3` | radio off, LPM3     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256510000&to=1487256590000) |          1.80mA |
| `ref_radiooff_lpm4` | radio off, LPM4     | [link](http://wsn-testbed.it.uu.se:3000/dashboard/db/overview-of-the-20-nodes?from=1487256650000&to=1487256730000) |          1.75mA |

## Timer A

* 32 kHz (ACLK), continuous mode
* never stopped, rolls over ever 2s

| interrupt      | description |
|----------------|-------------|
| `CCR1` compare | Used to calibrate ~5 MHz clock speed: how many subticks in 7 ticks?. Fires 32*7 ticks (6836us), compare TBR against last value of TBR. Then `value<<5` is the number of subticks in 7 ticks |
| `CCR2` compare | Used to trigger light sensor sampling. Fires every 100 ticks (~3050us) |
| `overflow`     | _enabled by default, but no action_ |

## Timer B

* 5 MHz (SMCLK), continuous mode
* never stopped (TODO: needs fixing)

| interrupt        | description |
|------------------|-------------|
| `CCR1` capture   | Triggered by the SFD pin toggling, triggers an interrupt. Do nothing on rising edge (start of frame). On falling edge (end of frame), read packet and schedule data transmission, if appropriate. |
| `CCR2` compare   | Fires when it's time to forward the packet, around 213.5us after end of frame of the ACK |
| `overflow`       | _enabled by default, but no action_ |

## pins

| pin  | description |
|------|-------------|
| P2.3 | output pin on the sink node (possibly on all nodes) |
| P3.4 | high during Timer A ISR |
| P6.6 | high during Timer B ISR |
| P2.6 | toggle at beginning of calibration |
| P3.5 | toggle when mote decides to retransmit (after ACK) |
| P6.7 | TODO |
