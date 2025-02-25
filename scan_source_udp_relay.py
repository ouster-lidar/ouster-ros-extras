import time
import socket
import threading
import json
from flask import Flask, request, jsonify
from ouster.sdk.client import SensorInfo
from ouster.sdk._bindings.client import PacketType
from ouster.sdk.util.parsing import scan_to_packets
import ouster.sdk.io_type
from ouster.sdk.io_type import OusterIoType


app = Flask(__name__)

class OsfPacketSource():

    def __init__(self, source_url, cycle):
        from ouster.sdk import open_source
        self._scan_source = open_source(source_url, cycle=cycle)

    @property
    def metadata(self) -> SensorInfo:
        return self._scan_source.metadata

    def __iter__(self):
        for scan in self._scan_source:
            packets = scan_to_packets(scan, self.metadata)
            for packet in packets:
                yield packet


class BagPacketSource():

    def __init__(self, source_url, cycle):
        from ouster.sdk.bag import BagPacketSource
        from ouster.sdk.client.packet_source_adapter import PacketSourceAdapter
        self._multi_packet_source = BagPacketSource(source_url)
        self._packet_source = PacketSourceAdapter(self._multi_packet_source, 0)

    @property
    def metadata(self):
        return self._packet_source.metadata

    def __iter__(self):
        for packet in self._packet_source:
            yield packet

class PcapPacketSource():

    def __init__(self, source_url, cycle):
        from ouster.sdk.pcap import PcapMultiPacketReader
        self._packet_source = PcapMultiPacketReader(source_url).single_source(0)

    @property
    def metadata(self):
        return self._packet_source.metadata

    def __iter__(self):
        for packet in self._packet_source:
            yield packet


class ScanSourceUdpRelay():

    io_type_handlers = {
        # OusterIoType.SENSOR: SensorScanSource,    TODO
        OusterIoType.PCAP: PcapPacketSource,
        OusterIoType.OSF: OsfPacketSource,
        OusterIoType.BAG: BagPacketSource,
    }

    def __init__(self, source_url):

        source_type = ouster.sdk.io_type.io_type(source_url)
        pkt_src_handler = ScanSourceUdpRelay.io_type_handlers[source_type]
        self._packet_source = pkt_src_handler(source_url, cycle=True)
        self._metadata = self._packet_source.metadata
        self._metadata_json_str = self._metadata.to_json_string()
        self._metadata_json_obj = json.loads(self._metadata_json_str)

        @app.route('/api/v1/sensor/metadata', methods=['GET'])
        def get_all_metadata():
            return jsonify(self._metadata_json_obj)

        @app.route('/api/v1/sensor/metadata/<metadata_type>', methods=['GET'])
        def get_sensor_metadata(metadata_type):
            if metadata_type in self._metadata_json_obj:
                print("returning")
                print({metadata_type: self._metadata_json_obj[metadata_type]})
                return jsonify(self._metadata_json_obj[metadata_type])
            return jsonify({"error": "Metadata not found"}), 404

        @app.route('/api/v1/sensor/config', methods=['GET', 'POST', 'DELETE'])
        def sensor_config_handler():
            if request.method == 'GET':
                return jsonify(self._metadata_json_obj["config_params"])
            elif request.method == 'POST':
                data = request.json
                # sensor_config.update(data)
                return jsonify({"message": "Configuration updated successfully"})
            elif request.method == 'DELETE':
                # sensor_config.clear()
                return jsonify({"message": "Configuration deleted successfully"})


        @app.route('/api/v1/sensor/cmd/get_config_param', methods=['GET'])
        def get_config_param():
            config_parameters = {
                "active": self._metadata_json_obj["config_params"],
                "staged": self._metadata_json_obj["config_params"]
            }
            args = request.args.get('args')
            if args in config_parameters:
                return jsonify({args: config_parameters[args]})
            else:
                return jsonify({"error": "Config parameter not found"}), 404
            
        @app.route('/api/v1/sensor/cmd/set_udp_dest_auto', methods=['GET'])
        def set_udp_dest_auto():
            return "{}"
        
        @app.route('/api/v1/sensor/cmd/set_config_param', methods=['GET', 'POST', 'DELETE'])
        def set_config_param():
            args = request.args.get('args')
            if len(args) > 0:
                return "\"set_config_param\""
            else:
                return jsonify({"error": "Config parameter not found"}), 404
            
        @app.route('/api/v1/user/data', methods=['GET', 'POST', 'DELETE'])
        def user_data():
            return "\"set_config_param\""

        @app.route('/api/v1/sensor/cmd/reinitialize', methods=['GET'])
        def reinitialize():
            return "{}"

        @app.route('/api/v1/system/firmware', methods=['GET'])
        def get_firmware_version():
            return jsonify({"fw": self._metadata_json_obj["sensor_info"]["image_rev"]})
        
        def run_flask_app():
            app.run(debug=True, port=80, use_reloader=False)
        
        # run_flask_app()
        self._flask_thread = threading.Thread(target=run_flask_app)
        self._flask_thread.daemon = True  # Allow the main thread to exit even if the Flask thread is still running
        self._flask_thread.start()
        self.start()

    def start(self):

        connected = False
        sock_lidar = 0
        sock_imu = 0
        while not connected:
            try:
                sock_lidar = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock_imu = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print("connected")
                connected = True
            except ConnectionRefusedError:
                print("connection refused")
                time.sleep(0.1)

        for packet in self._packet_source:
            if packet.type() == PacketType.Lidar:
                sock_lidar.sendto(packet.buf, ('127.0.0.1', self._metadata.config.udp_port_lidar))  # Send the chunk
            elif packet.type() == PacketType.Imu:
                sock_lidar.sendto(packet.buf, ('127.0.0.1', self._metadata.config.udp_port_imu))  # Send the chunk
            
            ## TODO: fix the time
            time.sleep(0.0015625)

        sock_lidar.close()
        sock_imu.close()

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("source_url")
    args = parser.parse_args()


    g = ScanSourceUdpRelay(args.source_url)
