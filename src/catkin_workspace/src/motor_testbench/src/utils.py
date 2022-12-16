import signal
import sys

# !!! Need to clarify this
def exit_gracefully(signum, frame):
	signal.signal(signal.SIGINT, original_sigint)
	sys.exit(1)
	signal.signal(signal.SIGINT, exit_gracefully)

def float_to_uint(number,v_min,v_max,bits):
    number = number/(v_max-v_min)*2**bits
    if (v_min != 0):
        number = int(number + 2**bits/2)
    else:
        number = int(number)
    return number
