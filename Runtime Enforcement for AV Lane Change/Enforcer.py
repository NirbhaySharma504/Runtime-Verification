############################## imports ###############################
import Automata
#####################################################################

############################################################################
# Compute and return automaton B_varphi for any given automaton A_varphi
# (Returns DFA that accepts all extensions of words accepted by the DFA phi provided as input.)
############################################################################
def getAutB(phi):
    deltaDict = {}
    finalSet = set()

    for q in phi.Q:
        if phi.F(q):
            finalSet.add(q)

    for q in phi.Q:
        for a in phi.S:
            if q in finalSet:
                deltaDict[(q, a)] = q
            else:
                deltaDict[(q, a)] = phi.d(q, a)

    return Automata.DFA(
        phi.S,
        phi.Q,
        phi.q0,
        lambda q: q in finalSet,
        lambda q, a: deltaDict[(q, a)],
    )


#############################################################################################################
###### Function to pre-compute emptiness check for each state in the automaton C= A_varphi*neg(B_varphi). ##
###### Input: Automaton C = A_varphi*neg(B_varphi) #########################################################
###### Output: A dictionary containing an entry for each state in the product C.#############################
######   (For each state, if the language accepted from the considered state is empty, #######################
######   then the value corresponding to that state is true and the value is false otherwise.)################
##############################################################################################################
def computeEmptinessDict(autC):
    dictEnf = {}
    for state in autC.Q:
        autC.makeInit(state)
        if autC.isEmpty():
            dictEnf[state] = True
        else:
            dictEnf[state] = False
    return dictEnf


################################################################################
################################################################################
###Enforcer takes two DFA A_psi, A_varphi and an input sequence of events#####
####Computes the output sequence sigmaS incrementally.##########################
################################################################################
################################################################################
def enforcer(psi, phi):
    sigmaC = []
    sigmaS = []

    p = psi.q0
    q = phi.q0

    autB = getAutB(phi)
    autB.complement()

    autC = Automata.DFAProduct([psi, autB], lambda o: o[0] and o[1]).getDFA()

    dictEnf = computeEmptinessDict(autC)

    # Updated exit command from "exit" to "end"
    print("Enter events one by one (i, safe, unsafe, e). Type 'end' to stop.")

    while True:
        event = input("Enter event: ").strip()

        # Ends loop on the trace "end"
        if event == "end":
            break

        print("event is.." + str(event))

        p = psi.step1(event)
        q = phi.step1(event)

        isEmpty = dictEnf[(p, q)]
        action = "stay"  # Default signal is to stay

        if isEmpty:
            # We are safe to release buffer
            for a in sigmaC:
                sigmaS.append(a)
            sigmaS.append(event)
            sigmaC = []
            
            if phi.F(q):
                action="change"
        else:
            # We are buffering, default action is stay
            sigmaC.append(event)
            action = "stay"
        print("Current state in psi: " + str(sigmaC))
        print("output sigmaS is.." + str(sigmaS))
        print("Action stream output: " + action + "\n")