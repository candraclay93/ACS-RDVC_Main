# src/lib-test/test_dummy_lib.py

# This is an ABSOLUTE import.
# It explicitly starts from the 'src' top-level package.
from src.lib.dummy_lib import Dumb

def run_test():
    print("Running test_dummy_lib.py using absolute import...")
    dumb = Dumb()
    dumb.name() # Assuming Dumb has an identity() method
    print("Test finished.")

if __name__ == "__main__":
    run_test()