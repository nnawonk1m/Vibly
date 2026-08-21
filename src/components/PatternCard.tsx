import { BLEService } from '@/services/ble/BleManager';
import { Image, Text, TouchableOpacity, View } from 'react-native';
import { COLORS, TYPOGRAPHY } from '../constants/theme';

type pattern = {
    name: string;
    id: string;
};

const PatternCard = (props: pattern) => {
    return (
        <View 
        style = {{
            backgroundColor: COLORS.secondary,
            flexDirection: 'row',
            padding: 10,
            justifyContent: 'space-between',
            alignItems: 'center',
            borderRadius: 10,
            marginBottom: 10,
            paddingHorizontal: 20
        }}>
            <Text style=
            {[
                TYPOGRAPHY.h2, 
                { color: COLORS.primaryText }]}
            >{props.name}</Text>
            <TouchableOpacity onPress={() => BLEService.write(props.id)}>
                <Image
                    source={require('../../assets/images/play-circle.png')}
                    style={{width: 30, height: 30, marginLeft: 60}}
                />
            </TouchableOpacity>
        </View>
    );
};

export default PatternCard;