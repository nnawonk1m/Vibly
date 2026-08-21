import { Alert, PermissionsAndroid, Platform } from "react-native";

export async function requestBluetoothPermission(): Promise<boolean> {
  if (Platform.OS === "ios") {
    return true;
  }

  const apiLevel = Number(Platform.Version);

  // Android 11 and below
  if (apiLevel < 31) {
    Alert.alert(
      "Unsupported Android Version",
      "This app requires Android 12 or later for Bluetooth scanning."
    );
    return false;
  }

  // Android 12+
  const result = await PermissionsAndroid.requestMultiple([
    PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
    PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
  ]);

  const granted =
    result["android.permission.BLUETOOTH_SCAN"] ===
      PermissionsAndroid.RESULTS.GRANTED &&
    result["android.permission.BLUETOOTH_CONNECT"] ===
      PermissionsAndroid.RESULTS.GRANTED;

  if (!granted) {
    Alert.alert(
      "Bluetooth Permission Required",
      "Please allow Bluetooth permissions to scan for and connect to your Vibly device."
    );
  }

  return granted;
}