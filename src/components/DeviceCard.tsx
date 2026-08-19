import { Image, Text, View } from 'react-native';
import { COLORS, TYPOGRAPHY } from '../constants/theme';

type device = {
    name: string;
}

const DeviceCard = (props: device) => {
    return (
        <View
        style={{
            flexDirection: 'row',
            justifyContent: 'center',
            alignItems: 'center'
        }}
        >
            <Image
                source={require('../../assets/images/bluetooth.png')}
                style={{ width: 10, height: 10 }}
            />
            <Text
            style={[
                TYPOGRAPHY.h3,
                { color: COLORS.placeholderText }
            ]}
            >{props.name}</Text>
        </View>

    );
}

export default DeviceCard;