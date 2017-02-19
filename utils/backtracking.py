L_D = 0.000320
D_D = 0.000677
L_A = 0.000192
D_A = 0.000590

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

output = []
for (k,v) in enumerate(backtracking):
    output += ['{0:<27} = {1:>4}; // {2:>7}us'.format('app_vars.backtracking[{0}]'.format(k),int(v*32768.0),int(1000000*v))]
output = '\n'.join(output)

print output

raw_input('press enter to close.')