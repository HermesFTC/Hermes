import { useSpring, animated } from '@react-spring/web'
import { easings } from '@react-spring/web'

export default function MyComponent() {
  const springs = useSpring({
    config: { duration: 1000, easing: easings.easeOutCubic  },
    from: { x: -100, opacity: 0 },
    to: { x: 0, opacity: 1 },
  })

  return (
    <animated.div
      style={{
        ...springs,
      }}
    >
        <button className="mx-auto p-4 border-hermes-cyan-dark border-4 border-solid rounded-3xl hover:scale-125 transition duration-500">
            <a href="/hermes/getting-started">
                <h1 className="text-5xl">Get Started</h1>
            </a>
        </button>

    </animated.div>
  )
}