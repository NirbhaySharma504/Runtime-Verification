from aalpy.SULs import SUL
from aalpy.learning_algs import run_Lstar
from aalpy.oracles import WMethodEqOracle
from aalpy.utils import save_automaton_to_file

class EvenParitySUL(SUL):
    def __init__(self):
        super().__init__()
        self.even_zeros = True
        self.even_ones = True
    def pre(self):
        self.even_zeros = True
        self.even_ones = True
    def post(self):
        pass
    def step(self, action):
        if action == '0':
            self.even_zeros = not self.even_zeros
        elif action == '1':
            self.even_ones = not self.even_ones
        return self.even_zeros and self.even_ones

alphabet = ['0', '1']
sul = EvenParitySUL()

# WMethodEqOracle targets systematic coverage of state space
# Set max_number_of_states to 4 (DFA for this task)
oracle = WMethodEqOracle(alphabet, sul, max_number_of_states=4)

learner = run_Lstar(alphabet, sul, oracle, automaton_type='dfa', print_level=0)
print(f"[WMethodEqOracle] Number of states: {len(learner.states)}")

save_automaton_to_file(learner, path='even_parity_dfa_WMETHOD', file_type='pdf')
print("Saved DFA to even_parity_dfa_WMETHOD.pdf")

