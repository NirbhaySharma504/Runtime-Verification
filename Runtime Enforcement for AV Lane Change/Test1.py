######## imports ###########
from pathlib import Path
import sys
import copy

sys.path.append(str(Path(__file__).resolve().parent))

import Enforcer
import Automata
#########################


#########################################################################################
### Define DFA describing input property psi ############################################
#########################################################################################
psi = Automata.DFA(
    ['i', 'safe', 'unsafe', 'e'],   # alphabet
    ['s0', 's1', 's2', 's3'],       # states
    's0',                           # start state
    lambda q: q in ['s0'],          # accepting states
    lambda q, a: {
        ('s0', 'i'): 's1',
        ('s0', 'safe'): 's0',
        ('s0', 'unsafe'): 's0',
        ('s0', 'e'): 's2',

        ('s1', 'i'): 's1',
        ('s1', 'safe'): 's3',
        ('s1', 'unsafe'): 's2',
        ('s1', 'e'): 's2',

        ('s2', 'i'): 's2',
        ('s2', 'safe'): 's2',
        ('s2', 'unsafe'): 's2',
        ('s2', 'e'): 's2',

        ('s3', 'i'): 's3',
        ('s3', 'safe'): 's3',
        ('s3', 'unsafe'): 's2',
        ('s3', 'e'): 's0',
    }[(q, a)]
)
#learned

###############################################################################
### Define DFA describing property to enforcer phi ############################
###############################################################################
phi = Automata.DFA(
    ['i', 'safe', 'unsafe', 'e'],              # alphabet
    ['s0', 's1', 's2', 's3','s4'],                  # states
    's0',                                      # start state
    lambda q: q in ['s4'],                     # accepting states (only q0 has a double circle)
    lambda q, a: {
        ('s0', 'i'): 's1',
        ('s0', 'safe'): 's0',
        ('s0', 'unsafe'): 's0',
        ('s0', 'e'): 's3',

        ('s1', 'i'): 's1',
        ('s1', 'safe'): 's2',
        ('s1', 'unsafe'): 's1',
        ('s1', 'e'): 's3',

        ('s2', 'i'): 's2',
        ('s2', 'safe'): 's2',
        ('s2', 'unsafe'): 's1',
        ('s2', 'e'): 's4',

        ('s3', 'i'): 's3',
        ('s3', 'safe'): 's3',
        ('s3', 'unsafe'): 's3',
        ('s3', 'e'): 's3',

        ('s4','i'):'s4',
        ('s4','safe'):'s4',
        ('s4','unsafe'):'s4',
        ('s4','e'):'s4'
    }[(q, a)]
)
#given to enforce

###############################################################################
### Invoke the enforcer with properties psi, phi, and some test input sequence
###############################################################################

if __name__ == '__main__':
    Enforcer.enforcer(copy.copy(psi), copy.copy(phi))
