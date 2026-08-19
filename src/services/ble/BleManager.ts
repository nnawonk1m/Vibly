import { BleManager, Device } from 'react-native-ble-plx';

class BLEServiceInstance {
  private manager: BleManager
  private device: Device | null = null
  private scanning = false;
  private connected = false;
  private scanTimeout: ReturnType<typeof setTimeout> | null = null;
  private connectionListener?: (connected: boolean) => void;
  private devices = new Map<string, Device>();
  private onDevicesChanged?: (devices: Device[]) => void;

  constructor() {
    this.manager = new BleManager()
  }

  async scan(
    onConnect: (device: Device) => void,
    onTimeout?: () => void
  ) {
    await this.disconnect();
    this.devices.clear();
    this.onDevicesChanged?.([]);

    if (this.scanning) {
        this.stopScan();
    }

    this.scanning = true;

    this.manager.startDeviceScan(null, null, async (error, device) => {
      if (error) {
      console.log("BLE ERROR OBJECT:", JSON.stringify(error, null, 2));
      console.log("Error code:", error.errorCode);
      console.log("Reason:", error.reason);
      console.log("Message:", error.message);

      this.stopScan();
      onTimeout?.();
      return;
    }
      if (!device) return;

      if (!this.devices.has(device.id)) {
        this.devices.set(device.id, device);

        console.log("New device:", device.name);

        this.onDevicesChanged?.([...this.devices.values()]);
      }

      if (device.name !== "Vibly") {
        return;
      }

      this.stopScan();
      const success = await this.connect(device);
      if(success) {
        onConnect(device)
      } else {
        onTimeout?.();
      }
    });

    this.scanTimeout = setTimeout(() => {
      this.stopScan();
      console.log("Timeout");
      onTimeout?.();
    }, 30000);
  }

  stopScan() {
    this.manager.stopDeviceScan();

    if (this.scanTimeout) {
      clearTimeout(this.scanTimeout);
      this.scanTimeout = null;
    }

    this.scanning = false;
  };

  isScanning() {
    return this.scanning;
  }

  async connect(device: Device): Promise<boolean> {
    try {
      const connection = await device.connect();
      await connection.discoverAllServicesAndCharacteristics();
      connection.onDisconnected((error, device) => {
          console.log("Disconnected");

          this.device = null;
          this.connected = false;

          this.connectionListener?.(false);
      });
      this.device = connection;
      this.connected = true;
      this.connectionListener?.(true);

      console.log("connection successful");
      return true;
    } catch(error) {
      console.log("connection failed: error occured");
      return false;
    }
  }

  async disconnect() {
      if (this.device) {
          try {
              await this.device.cancelConnection();
          } catch (e) {
              console.log(e);
          }

          this.device = null;
          this.connected = false;
          this.connectionListener?.(false);
      }
  }

  async write(val: string) {
    if(!this.device) return;

    await this.device.writeCharacteristicWithResponseForService(
      "780bf242-ff64-47f4-9627-cc76108e2a27",
      "38ff9161-1378-4414-a3c2-cb5ffc17ff92",
      btoa(val)
    );
  }

  setConnectionListener(listener: (connected: boolean) => void) {
    this.connectionListener = listener;
  }

  setOnDevicesChanged(callback: (devices: Device[]) => void) {
      this.onDevicesChanged = callback;
  }

  isConnected() {
    return this.connected;
  }
}

export const BLEService = new BLEServiceInstance()