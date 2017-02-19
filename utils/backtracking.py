'''
Utility to calculate the backtracking durations for each hop.

Run this utility and copy-paste its output into the flashflood
project file.

\author Tengfei Chang <poipoi>, February 2017.
\author Thomas Watteyne <thomas.watteyne@inria.fr>, February 2017.
'''

# atomic durations (measured using a logic analyzer)
L_D = 0.000320 # length of a data packet, from SFD high to SFD low
D_D = 0.000677 # duration of a data packet, from SFD high until the SFD high of the subsequent ACK
L_A = 0.000192 # length of an ACK packet, from SFD high to SFD low
D_A = 0.000590 # duration of an ACK packet, from SFD high until the SFD high of the subsequent DATA

# per-hop backtracking value
backtracking = [
    0,
    L_D,
    D_D+L_A,
    D_D+D_A+L_D,
    D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+L_D,
    D_D+D_A+D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+D_D+D_A+L_D,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+L_D,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+L_D,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+L_D,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+L_A,
    D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+D_D+D_A+L_D,
]

output        = []
output       += ['Utility to calculate backtracking durations']
output       += ['Tengfei Chang, Thomas Watteyne']
output       += ['']
output       += ['======== start of code to copy-paste ======']
output       += ['']
output       += ['// DO NOT EDIT DIRECTLY!']
output       += ['// Generated automatically by the backtracking.py script.']
output       += ['const uint16_t backtracking[16] = {']
for (h,v) in enumerate(backtracking):
    output   += ['   {0:>4}, // hop {1:>2}, {2:>4}us'.format(int(v*32768.0),h,int(v*1000000))]
output       += ['};']
output       += ['']
output       += ['======== end of code to copy-paste ========']
output       += ['']
output        = '\n'.join(output)

print output

raw_input('Script ended normally. Press enter to close.')