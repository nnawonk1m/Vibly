import { COLORS, TYPOGRAPHY } from '@/constants/theme';
import { router } from 'expo-router';
import { Image, ImageSourcePropType, Text, TouchableOpacity } from 'react-native';

type navItem = {
    name: "home" | "manual" | "login";
    source: ImageSourcePropType;
    route: "/" | "/manual" | "/login";
    active: boolean;
}

const BottomNavItem = (props: navItem) => {
    const onPress = () => {
        router.replace(props.route);
    };

    return (
    <TouchableOpacity 
    style = {{
        flexDirection: 'column',
        justifyContent: 'center',
        alignItems: 'center',
        backgroundColor: COLORS.primary,
    }}
    onPress = {onPress}>
            <Image
                source = {props.source}
                style={{ width: 20, height: 20, tintColor: props.active ? COLORS.placeholderText : COLORS.secondary }}
            />
            <Text style={[ TYPOGRAPHY.h3, { color: props.active ? COLORS.placeholderText : COLORS.secondary } ]}>{props.name}</Text>
    </TouchableOpacity>
    );
}

export default BottomNavItem;