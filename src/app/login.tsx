import { SafeAreaView } from 'react-native-safe-area-context';
import SigninCard from "../components/SigninCard";
import { COLORS } from "../constants/theme";

export default function LoginLayout() {
    return (
        <SafeAreaView style={{
            flex: 1,
            justifyContent: 'center',
            alignItems: 'center',
            backgroundColor: COLORS.primary,
            paddingHorizontal: 16,
        }}>
            <SigninCard />
        </SafeAreaView>
    );
}