import requests
import json

BASE_URL = "http://localhost:8000"  # Assuming local run for testing

def test_health_check():
    """Tests the root endpoint."""
    response = requests.get(BASE_URL + "/")
    assert response.status_code == 200
    print("Health check passed.")

def test_chat_selected():
    """Tests the /chat/selected endpoint."""
    payload = {
        "question": "What is ROS 2?",
        "context": "ROS 2 is a set of software libraries and tools that help you build robot applications."
    }
    response = requests.post(BASE_URL + "/chat/selected", json=payload)
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    print("Chat selected endpoint test passed.")
    print("Response:", data)

def test_get_history():
    """Tests the /chat/history endpoint."""
    response = requests.get(BASE_URL + "/chat/history")
    assert response.status_code == 200
    data = response.json()
    assert "history" in data
    print("Get history endpoint test passed.")

if __name__ == "__main__":
    test_health_check()
    test_chat_selected()
    test_get_history()
    print("All tests passed!")
