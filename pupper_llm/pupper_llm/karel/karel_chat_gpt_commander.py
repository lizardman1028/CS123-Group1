import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
from openai import OpenAI
import karel  # Importing your KarelPupper API

client = OpenAI(api_key='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')  # Set your OpenAI API key here

class GPT4ConversationNode(Node):
    def __init__(self):
        super().__init__('gpt4_conversation_node')

        # Create a subscriber to listen to user queries
        self.subscription = self.create_subscription(
            String,
            'user_query_topic',  # Replace with your topic name for queries
            self.query_callback,
            10
        )

        # Create a publisher to send back responses
        self.publisher_ = self.create_publisher(
            String,
            'gpt4_response_topic',  # Replace with your topic name for responses
            10
        )

        self.get_logger().info('GPT-4 conversation node started and waiting for queries...')

        # Initialize the text-to-speech engine
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)  # Set the speed of speech (optional)

        # Initialize KarelPupper robot control
        self.pupper = karel.KarelPupper()

    # TODO: Implement the query_callback method
    # msg is a String message object that contains the user query. You can extract the query using msg.data
    def query_callback(self, msg):
        user_query = msg.data
        response = self.get_gpt4_response(user_query)

        response_msg = String()
        response_msg.data = response
        self.publisher_.publish(response_msg)
        # Paste in your implementation from simple_gpt_chat.py
        
        # Play the response through the speaker with the play_response method
        self.play_response(response)
        # Parse and execute robot commands if present with the execute_robot_command method
        self.execute_robot_command(response)

    def get_gpt4_response(self, query):
        try:
            # Making the API call to GPT-4 using OpenAI's Python client
            prompt = str("using the commands move() turn_left() turn_right() bark() and stop() to control a robot. Here is an example of some code which makes a robot walk forwards then turn around and walk back \n"
            +"pupper.move()\n"
            +"pupper.bark()\n"
            +"pupper.turn_right()\n"
            +"pupper.turn_right()\n"
            +"pupper.move()\n")

            response = client.chat.completions.create(model="gpt-4",  # Model identifier, assuming GPT-4 is used
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": query}
            ],
            max_tokens=150)  # Adjust token limit based on your requirement

            # Extract the assistant's reply from the response
            gpt4_response = response.choices[0].message.content
            return gpt4_response

        except Exception as e:
            self.get_logger().error(f"Error calling GPT-4 API: {str(e)}")
            return "Sorry, I couldn't process your request due to an error."

    def play_response(self, response):
        try:
            # Use the TTS engine to say the response out loud
            self.tts_engine.say(response)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"Error playing response through speaker: {str(e)}")

    def execute_robot_command(self, response):
        # Convert the response to lowercase to handle case-insensitivity
        response = response.lower()
        self.get_logger().info(f"Response: {response}")
        # TODO: Implement the robot command execution logic, in a large if-else statement. Your conditionals should be set based on the expected commands from GPT-4, and the corresponding methods should be called on the KarelPupper object.
        for line in response.split("\n"):
            line = line.strip()
            if "pupper.move()" in line:
                self.pupper.move()
            elif "pupper.bark()" in line:
                self.pupper.bark()
            elif "pupper.turn_left()" in line:
                self.pupper.turn_left()
            elif "pupper.turn_right()" in line:
                self.pupper.turn_right()
            elif "pupper.stop()" in line:
                self.pupper.stop()

        pass

def main(args=None):
    rclpy.init(args=args)

    # Create the node and spin it
    gpt4_conversation_node = GPT4ConversationNode()
    rclpy.spin(gpt4_conversation_node)

    # Clean up and shutdown
    gpt4_conversation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
