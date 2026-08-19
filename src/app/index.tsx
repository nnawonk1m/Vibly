import DeviceCard from '@/components/DeviceCard';
import { useEffect, useState } from 'react';
import { FlatList, Text } from 'react-native';
import { Device } from 'react-native-ble-plx';
import { SafeAreaView } from "react-native-safe-area-context";
import BottomNav from "../components/BottomNav";
import ScanButton from '../components/ScanButton';
import TopNav from "../components/TopNav";
import Wearable3D from '../components/Wearable3D';
import { COLORS, TYPOGRAPHY } from "../constants/theme";
import { BLEService } from "../services/ble/BleManager";
import { requestBluetoothPermission } from "../services/ble/BlePermissions";

type ScanStatus = "idle" | "scanning" | "connected" | "failed";

export default function IndexLayout() {
  const [status, setStatus] = useState<ScanStatus>("idle");
  const [connectedDeviceName, setConnectedDeviceName] = useState("");
  const [isConnected, setConnected] = useState(false);
  const [devices, setDevices] = useState<Device[]>([]);

  useEffect(() => {
    setConnected(BLEService.isConnected());
    
    BLEService.setConnectionListener(setConnected);
    BLEService.setOnDevicesChanged(setDevices);
  }, []);

  const handleScan = async () => {
    console.log("Button pressed");

    const granted = await requestBluetoothPermission();
    console.log("Granted:", granted);

    if(!granted) {
      return;
    } else {
      setStatus("scanning");

      await BLEService.scan(
        (device) => {
          setStatus("connected");
          setConnectedDeviceName(device.name || "");
        },
        () => {setStatus("failed");}
      );
    }
  }

  return (
    <SafeAreaView 
    style={{
      flex: 1,
      backgroundColor: COLORS.bg,
      alignItems: "center",
    }}>
      <TopNav connected={isConnected}/>
      <Wearable3D style={{ width: '100%', height: 400 }} />
      <ScanButton onPress={handleScan}/>

      {status === "scanning" && 
      <Text style={[ TYPOGRAPHY.h3, { color: COLORS.placeholderText, marginVertical: 20 }]}>Scanning...</Text>}
      
      {status === "failed" && 
      <Text style={[ TYPOGRAPHY.h3, { color: COLORS.placeholderText, marginVertical: 20 }]}>Scanning failed.</Text>}
      
      {status === "connected" && 
      <Text style={[ TYPOGRAPHY.h3, { color: COLORS.placeholderText, marginVertical: 20 }]}>Connected to {connectedDeviceName}</Text>}

      <FlatList
        data={devices}
        keyExtractor={(item) => item.id}
        renderItem={({ item }) => (
          <DeviceCard name={item.name ?? "Unknown Device"}/>
        )}
      />

      <BottomNav 
        homeRoute = {true}
        manualRoute = {false}
      />
    </SafeAreaView>);
}
