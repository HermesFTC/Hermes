import MyComponent from "@/components/TestComponent"

export default function Home() {
    return (
        <div className="text-center h-[100vh] text-hermes-cyan-dark">
            <div className="flex flex-row m-auto items-center justify-center">
                <img src="src/assets/icons/hermes_logo.svg"/>
                <h1 className="text-9xl">HermesFTC</h1>
            </div>
            <div className="text-center w-full">
                <MyComponent></MyComponent>
            </div>
        </div>
    )
}