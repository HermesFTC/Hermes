import { useEffect, useState } from "react";


export default function ProgressBar() {
 
    const [currentIndex, setCurrentIndex] = useState(localStorage.getItem('TuningProgress'))

    if (currentIndex == null) {
        setCurrentIndex("0")
    }

    let idx = parseInt(currentIndex as string)

    let totalIndex = 0;
    switch(localStorage.getItem("TuningStage")) {
        case "Localizer":
            totalIndex = 3;
            break;
        case "Drive":
            totalIndex = 2;
            break;
        case "SysId":
            totalIndex = 4;
            break;
    }

    const [percent, setPercent] = useState(idx * 100 / totalIndex);

    useEffect(() => {
        function handleStorage() {
            setCurrentIndex(localStorage.getItem('TuningProgress'))
            setPercent(parseInt(currentIndex as string) * 100 / totalIndex)
        }
        window.addEventListener('storage', handleStorage);
        return () => {
          window.removeEventListener('storage', handleStorage);
        };
      }, []);

    return <div className="w-full items-center align-center flex justify-center mt-4 pb-4">
        <div className="border-solid border-black border-[2px] w-full mx-4 md:w-1/2 lg:w-1/3 h-4 rounded-xl">
        <div className={percent === 100 ? "bg-cyan-400" : "bg-green-400" + " h-full rounded-xl"} style={{width: percent.toString() + "%"}}/>
        </div>
    </div>
}