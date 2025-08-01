from flask import Flask, request, jsonify

app = Flask(__name__)

@app.route('/api/receive_data', methods=['POST'])
def receive_data():
    data = request.json
    print("Received data:", data)
    # Process the data as needed (e.g., save to a database, visualize, etc.)
    return jsonify({"status": "success"}), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
