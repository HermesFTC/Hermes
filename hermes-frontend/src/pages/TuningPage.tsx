import { GenericButton } from "@/components/GenericButton";
import { Outlet } from "react-router-dom";

export default function TuningPage() {

    /*const printDashDebug = useSelector(
        (state: RootState) => state.config.configRoot,
    ) as CustomVarState;*/
      
    return (
    <div className="items-center text-center justify-center lg:mx-[30%] mx-8 mt-4">
        <Outlet/>

        {/* <GenericButton onClick=>Print Dash Debug</GenericButton> */}
    </div>
    )
}
