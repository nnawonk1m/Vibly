import { Text, View } from 'react-native';
import { COLORS, TYPOGRAPHY } from '../constants/theme';

type batteryProp = {
    percentage: number;
}

const Battery = (props: batteryProp) => {
    return (
        <View 
        style={{ 
            justifyContent: 'center', 
            alignItems: 'center', 
            width: 50, height: 20,
            borderRadius: 10,
            backgroundColor: COLORS.placeholderText
        }}
        >
            <View 
            style={{
                position: 'absolute',
                left: 0, top: 0, bottom: 0,
                width: `${props.percentage}%`, height: '100%',
                borderTopLeftRadius: 10, borderBottomLeftRadius: 10,
                backgroundColor: COLORS.bg
            }}
            ></View>
            <Text 
            style={[
                TYPOGRAPHY.h3,
                {
                    color: COLORS.primaryText,
                }
            ]}
            >
                {props.percentage}%
            </Text>
        </View>
    );
}

export default Battery;