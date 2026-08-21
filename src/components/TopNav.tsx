import { Text, View } from 'react-native';
import { COLORS, TYPOGRAPHY } from '../constants/theme';
import Battery from './Battery';

type connection = {
    connected: boolean;
}

const TopNav = (props: connection) => {
    return (
        <View
        style={{
            position: 'absolute',
            top: 0, left: 0, right: 0,
            flexDirection: 'row',
            justifyContent: 'space-between',
            alignItems: 'center',
            paddingTop: 30,
            paddingBottom: 20,
            paddingHorizontal: 20,
            backgroundColor: COLORS.primary
        }}
        >
            <Text
            style={[
                TYPOGRAPHY.h3,
                { color: COLORS.bg }
            ]}
            >{props.connected ? "Connected" : "Not connected"}</Text>
            <Battery percentage={60}></Battery>
        </View>
    );
}

export default TopNav;