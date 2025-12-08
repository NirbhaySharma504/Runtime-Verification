from aalpy.SULs import SUL
from aalpy.learning_algs import run_Lstar
from aalpy.oracles import RandomWalkEqOracle
from aalpy.utils import save_automaton_to_file

# SUL models DFA for even 0s AND even 1s
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
        return self.even_zeros and self.even_ones  # Accept if both counts are even

alphabet = ['0', '1']
sul = EvenParitySUL()

# Set up random walk equivalence testing
# Helps catch behavioral differences by exploring randomly
oracle = RandomWalkEqOracle(alphabet, sul, num_steps=1000, reset_prob=0.09)

learner = run_Lstar(alphabet, sul, oracle, automaton_type='dfa', print_level=0)
print(f"[RandomWalkEqOracle] Number of states: {len(learner.states)}")

# Export DFA to PDF for visual comparison
save_automaton_to_file(learner, path='even_parity_dfa_RANDOM', file_type='pdf')
print("Saved DFA to even_parity_dfa_RANDOM.pdf")

