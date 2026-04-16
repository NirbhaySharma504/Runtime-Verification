import sys
import copy
import random
from pathlib import Path

# Add the appropriate paths
sys.path.append(str(Path(__file__).resolve().parent))

# Import required modules from the workspace
import runtime_verifier
import Automata
import Enforcer
from sampler import generate_good_trace, generate_bad_trace, run_dfa, ACCEPT

try:
    from learner import CarlaProtocolSUL
    CARLA_AVAILABLE = True
except ImportError:
    CARLA_AVAILABLE = False
    print("[WARNING] CARLA module or associated dependencies not found. Mocking the driver behaviour for demonstration...")


class RuntimeVerifierManager:
    """Wrapper that evaluates step-by-step verification."""
    def __init__(self, psi, phi):
        self.psi = copy.copy(psi)
        self.phi = copy.copy(phi)
        
        self.p = self.psi.q0
        self.q = self.phi.q0
        
        self.autB = Enforcer.getAutB(self.phi)
        self.autB.complement()
        
        self.autC = Automata.DFAProduct([self.psi, self.autB], lambda o: o[0] and o[1]).getDFA()
        self.dictEnf = Enforcer.computeEmptinessDict(self.autC)
        self.sigmaC = []
        self.sigmaS = []

    def process_event(self, event):
        """Processes an event through the verifier and returns the determined action."""
        self.p = self.psi.step1(event)
        self.q = self.phi.step1(event)
        
        isEmpty = self.dictEnf[(self.p, self.q)]
        action = "stay"
        
        if isEmpty:
            for a in self.sigmaC:
                self.sigmaS.append(a)
            self.sigmaS.append(event)
            self.sigmaC = []
            
            if self.phi.F(self.q):
                action = "change"
        else:
            self.sigmaC.append(event)
            action = "stay"
            
        return action

def main():
    print("="*60)
    print(" LANE CHANGE VERIFIER INITIALIZED ".center(60))
    print("="*60)

    # 1. Generate trace from sampler
    is_good = random.choice([True, False])
    if is_good:
        trace = generate_good_trace()
        trace_type = "GOOD Trace (Should be SAFE)"
    else:
        trace = generate_bad_trace()
        trace_type = "BAD Trace (Throws UNSAFE / ERROR)"
        
    final_sampler_state = run_dfa(trace)
    sampler_outcome = "SAFE" if final_sampler_state in ACCEPT else "UNSAFE"
    
    print(f"\n[Sampler] Trace generation type: {trace_type}")
    print(f"[Sampler] Generated events: {trace}")
    print(f"[Sampler] Theoretical outcome: {sampler_outcome}\n")
    
    # 2. Setup the Enforcer and the Driver
    verifier = RuntimeVerifierManager(runtime_verifier.psi, runtime_verifier.phi)
    
    if CARLA_AVAILABLE:
        driver = CarlaProtocolSUL()
        print("[Driver] Booting up CARLA Driver Physics SUL...")
        driver.pre()
    else:
        driver = None

    carla_outcome = "SAFE LANE CHANGE"
    
    # 3. Simulate step by step with the trace from the sampler
    for event in trace:
        print(f"\n---> Sending event '{event}' to Verifier...")
        
        # Ask Verifier
        action = verifier.process_event(event)
        print(f"     Verifier decision: {action.upper()}")

        if CARLA_AVAILABLE:
            # Suppose Carla is available, we enforce the verifier output.
            # Only trigger lane changes or update carla if the verifier complies
            # If action is 'change', it implicitly means 'e' was verified safe!
            if action == 'stay' and event == 'e':
                print("     [Driver] Suppressing unverified 'e', enforcing 'stay' instead to avoid crash!")
            else:
                # Forward to CARLA SUL
                is_valid = driver.step(event)
                
                # Check for crash using the driver's internals
                if not is_valid or getattr(driver, '_sink_flag', False):
                    carla_outcome = "CRASH OCCURRED"
                    print("     [Driver] Physical impact / Protocol violation registered.")
        else:
            # Mock driver fallback behavior
            # If the trace is bad, let's mock it
            if action == 'change':
                print("     [Driver] EXECUTING LANE CHANGE IN SIMULATION.")
            elif action == 'stay' and event == 'e':
                print("     [Driver] INTERCEPTED DANGEROUS COMMAND: Enforcing 'stay' and blocking lane change.")
            else:
                print(f"     [Driver] Status: {action.upper()}")
                
    if CARLA_AVAILABLE:
        # Complete simulation safely
        if not getattr(driver, '_sink_flag', False) and carla_outcome != "CRASH OCCURRED":
            carla_outcome = "SAFE LANE CHANGE (Confirmed by CARLA SUL)"
            
        driver.post()
    else:
        # Mock final evaluation
        carla_outcome = "SAFE (Mocked Driver Output)" if sampler_outcome == "SAFE" else "BLOCKED/STAY (Mocked Driver Output)"

    print("="*60)
    print("FINAL EVALUATION".center(60))
    print("="*60)
    print(f"Original Trace given by sampler      : {trace}")
    print(f"Sampler Output                       : {sampler_outcome}")
    print(f"Simulated / Carla Output             : {carla_outcome}")
    print("="*60)

if __name__ == '__main__':
    main()
