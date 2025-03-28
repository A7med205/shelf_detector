import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import assemblyai as aai
import base64
import boto3
from botocore.exceptions import NoCredentialsError

class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')

        self.subscription = self.create_subscription(
            String,
            '/web_audio_input',
            self.audio_callback,
            10
        )

        self.publisher = self.create_publisher(Int32, '/command_topic', 10)
        self.get_logger().info("Voice Processor Node Initialized")

        # Configuring AWS and S3 details
        self.s3 = boto3.client(
            's3',
            aws_access_key_id='AKIAS2VS4XGUXM5YSU3Z',
            aws_secret_access_key='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',
            region_name='eu-north-1'
        )

        self.bucket_name = 'voiceinput--eun1-az1--x-s3'
        self.s3_key = '/audio_input.webm'  # Path in the S3 bucket

    def find_match(self, input_string):
        # Hardcoded array of strings
        keywords = ["stop", "search", "home"]

        # Convert input string to lowercase for case-insensitive comparison
        input_string_lower = input_string.lower()

        # Iterate through the keywords and return the index of the first match
        for index, keyword in enumerate(keywords):
            if keyword.lower() in input_string_lower:
                return index

        # Return -1 if no match is found
        return -1

    def audio_callback(self, msg):
        self.get_logger().info("Audio data received")
        try:
            # Extracting and decoding the base64 data (WebM format)
            base64_data = msg.data.split(",")[1]
            webm_data = base64.b64decode(base64_data)

            # Saving the WebM data to a file
            webm_file_path = "audio_input.webm"
            with open(webm_file_path, "wb") as webm_file:
                webm_file.write(webm_data)

            self.get_logger().info(f"WebM file saved at: {webm_file_path}")

            # Uploading file to S3
            try:
                self.s3.upload_file(webm_file_path, self.bucket_name, self.s3_key)
                self.get_logger().info("File uploaded to S3")
            except NoCredentialsError as e:
                self.get_logger().error("AWS credentials not available.")
                return

            # Generating a presigned URL for the file
            try:
                presigned_url = self.s3.generate_presigned_url(
                    'get_object',
                    Params={'Bucket': self.bucket_name, 'Key': self.s3_key},
                    ExpiresIn=3600  # 1 hour expiration
                )
                self.get_logger().info("Presigned URL generated")
            except Exception as e:
                self.get_logger().error(f"Error generating presigned URL: {e}")
                return

            # Setting AssemblyAI API Key
            aai.settings.api_key = "de04edad715d470785ddb27fd1b66c03"
            transcriber = aai.Transcriber()

            # Transcribing using the presigned URL
            transcript = transcriber.transcribe(presigned_url)

            if transcript.error:
                self.get_logger().error(f"Transcription error: {transcript.error}")
            else:
                command = transcript.text
                final_cmd = self.find_match(command)
                self.publish_command(final_cmd)

        except Exception as e:
            self.get_logger().error(f"Error processing audio data: {e}")

    def publish_command(self, final_cmd):
        final_cmd_msg = Int32()
        final_cmd_msg.data = final_cmd
        self.publisher.publish(final_cmd_msg)
        self.get_logger().info(f"Published command: {final_cmd}")

def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceProcessor()
    rclpy.spin(voice_processor)
    voice_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
