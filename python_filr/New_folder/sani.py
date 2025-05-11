import requests
import time

# Replace with your ESP8266's IP address
ESP8266_IP = "192.168.121.208"  # Example IP, update after checking Serial Monitor
BASE_URL = f"http://{ESP8266_IP}"

def send_command(command):
    try:
        # Send HTTP GET request to the appropriate endpoint
        response = requests.get(f"{BASE_URL}/{command}")
        if response.status_code == 200:
            print(f"Command {command} sent successfully: {response.text}")
        else:
            print(f"Failed to send command {command}. Status code: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to ESP8266: {e}")

def main():
    while True:
        # Get user input
        command = input("Enter command (1, 2, 3, 4, or 'q' to quit): ").strip()
        
        # Validate and send command
        if command in ['1', '2', '3', '4']:
            send_command(command)
        elif command.lower() == 'q':
            print("Exiting...")
            break
        else:
            print("Invalid command. Use '1', '2', '3', '4', or 'q'.")
        
        # Small delay to avoid overwhelming the server
        time.sleep(0.5)

if __name__ == "__main__":
    # Ensure the ESP8266 is connected to WiFi and note its IP
    print(f"Controlling ESP8266 at {BASE_URL}")
    main()