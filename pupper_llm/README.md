# Group 1 Final Project Code

Slides: https://docs.google.com/presentation/d/1PkUF8yBt4b4o_hlcP-XUiFB2-tKcdd_b2StPuGY-5MI/edit?usp=sharing

Our final project was for to use Pupper as a mechanism for artistic creation. Details can be found in the slides above.

Hardware changes includes:
- Adding the marker attachment to the upper
  - Custom-designed Lego joint, enabling precise movement and control of marker
  - Rubber bands made it so that marker was always in contact with the floor
  - Drilled holes through Pupper's sides to secure the marker while it walks/turns
- Soldering an arduino board with a button that can be used to trigger Pupper listening for what to draw
- Mounting speaker and arduino board to the Pupper so that it can walk around on its own without these getting in the way

Software changes include:
- whisper_ping.py:
  - Now waits in an infinite loop for serial input from the arduino button to trigger listening + whisper API
  - Plays audio to ask for instructions and when it understands those instructions
- karel_chat_gpt_commander.py:
  - Updated prompt to include new Karel API methods + an example of drawing a few shapes
  - Updated code to parse the API output to handle calls to the new methods
  - Added logic to play audio once the Pupper is done drawing
- karel.py
  - Added a move method that moves at a specified constant rate that we experimentally determined to produce the straightest lines
  - Added a turn method that took in radians as input, tuned this method to include a small amount of walking forward for straighter lines

 
  
 


