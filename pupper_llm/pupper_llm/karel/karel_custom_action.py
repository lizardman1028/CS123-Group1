# karel_test.py

import karel
import time


def main():
    pupper = karel.KarelPupper()
    time.sleep(5)
    pupper.move()
    pupper.bark()
    pupper.turn_right()
    pupper.turn_right()
    pupper.move()
    

if __name__ == '__main__':
    main()
