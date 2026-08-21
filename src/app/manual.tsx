import PatternCard from "@/components/PatternCard";
import { BLEService } from '@/services/ble/BleManager';
import { useEffect, useState } from 'react';
import { Text, TouchableOpacity, View } from "react-native";
import { SafeAreaView } from "react-native-safe-area-context";
import BottomNav from "../components/BottomNav";
import TopNav from "../components/TopNav";
import { COLORS, TYPOGRAPHY } from "../constants/theme";

export default function ManualLayout() {
    const [isConnected, setConnected] = useState(false);
    const title = "Haptic Patterns";
    const content = "Vibly alerts the users of human speech and other emergency audio cues including car honks and baby cries. Click on the play button to test each vibration pattern specific to the categories identified."

    useEffect(() => {
        setConnected(BLEService.isConnected());
        
        BLEService.setConnectionListener(setConnected);
    }, []);

    return (
        <SafeAreaView 
        style={{
        flex: 1,
        justifyContent: 'flex-start',
        alignItems: 'flex-start',
        backgroundColor: COLORS.bg
        }}>
            <TopNav connected={isConnected}/>
            <Text style={[ TYPOGRAPHY.h1, { color: COLORS.primaryText, marginHorizontal: 16, marginTop: 50 }]}>{title}</Text>
            <Text style={[ TYPOGRAPHY.h2, { color: COLORS.placeholderText, marginHorizontal: 16 }]}>{content}</Text>
            <View 
            style={{
                marginHorizontal: 16,
                marginVertical: 20,
            }}>
                <PatternCard
                    name="Speech"
                    id="1"
                /> 
                <PatternCard
                    name="Car honks"
                    id="2"
                /> 
                <PatternCard
                    name="Baby cries"
                    id="3"
                />
                <TouchableOpacity 
                onPress={() => BLEService.write("0")}
                style = {{
                    justifyContent: "center",
                    alignItems: "center",
                    backgroundColor: COLORS.primary,
                    paddingHorizontal: 10,
                    paddingVertical: 10,
                    marginHorizontal: 10,
                    marginVertical: 0,
                    borderRadius:20,
                    minWidth: 120
                }}
                >
                    <Text style={[ TYPOGRAPHY.h2, { color: COLORS.bg } ]}>stop</Text>
                </TouchableOpacity>
            </View>
            <BottomNav 
            homeRoute = {false}
            manualRoute = {true}
            />
        </SafeAreaView>);
}
