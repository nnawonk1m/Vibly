import { COLORS, TYPOGRAPHY } from '@/constants/theme';
import { Text, TouchableOpacity } from 'react-native';

type ScanButtonProps = {
    onPress: () => void;
};

const ScanButton = (props: ScanButtonProps) => {
    return (
    <TouchableOpacity 
    onPress={props.onPress}
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
        <Text style={[ TYPOGRAPHY.h2, { color: COLORS.bg } ]}>scan</Text>
    </TouchableOpacity>
    );
}

export default ScanButton;