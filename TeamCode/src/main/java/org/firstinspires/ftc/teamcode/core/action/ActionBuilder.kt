package org.firstinspires.ftc.teamcode.core.action

import org.firstinspires.ftc.teamcode.core.Module
import org.firstinspires.ftc.teamcode.core.state.State
import java.lang.Runnable
import java.util.function.Supplier
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.withTimeoutOrNull

class ActionBuilder {
    private val steps = mutableListOf<Action.Step>()
    private val targets = mutableSetOf<Module>()
    private var name = "Action"

    fun set(vararg states: State): ActionBuilder {
        states.forEach { state ->
            if (state.module != null) {
                targets.add(state.module!!)
            } else {
                System.err.println("[ActionBuilder] Warning: State ${state.javaClass.simpleName} has no module attached. " +
                    "Ensure the state is registered with a Module before using it in actions.")
            }
            steps.add(Action.Step { state.apply() })
        }
        return this
    }

    /** Evaluate state suppliers at execution time rather than build time. */
    @SafeVarargs
    fun setLazy(vararg suppliers: Supplier<out State>): ActionBuilder {
        steps.add(Action.Step {
            for (supplier in suppliers) supplier.get().apply()
        })
        return this
    }

    fun run(code: Runnable): ActionBuilder {
        steps.add(Action.Step { code.run() })
        return this
    }

    fun run(code: () -> Unit): ActionBuilder {
        steps.add(Action.Step { code() })
        return this
    }

    fun runSuspending(code: suspend () -> Unit): ActionBuilder {
        steps.add(Action.Step { code() })
        return this
    }

    fun delay(ms: Long): ActionBuilder {
        steps.add(Action.Step { kotlinx.coroutines.delay(ms) })
        return this
    }

    fun waitUntil(condition: () -> Boolean): ActionBuilder = waitUntil(condition, Long.MAX_VALUE)

    fun waitUntil(condition: () -> Boolean, timeoutMs: Long): ActionBuilder {
        steps.add(Action.Step {
            withTimeoutOrNull(timeoutMs) {
                while (!condition()) kotlinx.coroutines.delay(10)
            }
        })
        return this
    }

    fun waitWhile(condition: () -> Boolean): ActionBuilder = waitUntil({ !condition() })

    fun waitWhile(condition: () -> Boolean, timeoutMs: Long): ActionBuilder =
        waitUntil({ !condition() }, timeoutMs)

    /** Inline another action's steps and targets. */
    fun action(action: Action): ActionBuilder {
        targets.addAll(action.targets)
        steps.addAll(action.steps)
        return this
    }

    /** Skip remaining steps if {@code condition} is true. */
    fun stopIf(condition: () -> Boolean): ActionBuilder {
        steps.add(Action.Step { parent ->
            if (condition()) parent.completeEarly()
        })
        return this
    }

    fun ifThen(condition: () -> Boolean, ifTrue: Action): ActionBuilder {
        targets.addAll(ifTrue.targets)
        steps.add(Action.Step {
            if (condition()) ifTrue.runSuspending()
        })
        return this
    }

    fun ifElse(condition: () -> Boolean, ifTrue: Action, ifFalse: Action): ActionBuilder {
        targets.addAll(ifTrue.targets)
        targets.addAll(ifFalse.targets)
        steps.add(Action.Step {
            if (condition()) ifTrue.runSuspending() else ifFalse.runSuspending()
        })
        return this
    }

    fun parallel(vararg actions: Action): ActionBuilder {
        actions.forEach { targets.addAll(it.targets) }
        steps.add(Action.Step {
            coroutineScope {
                actions.map { a ->
                    a.reset()
                    async { a.runSteps() }
                }.awaitAll()
            }
        })
        return this
    }

    fun race(vararg actions: Action): ActionBuilder {
        actions.forEach { targets.addAll(it.targets) }
        steps.add(Action.Step {
            coroutineScope {
                val deferreds = actions.map { a ->
                    a.reset()
                    async { a.runSteps() }
                }
                select<Unit> {
                    deferreds.forEach { it.onAwait { } }
                }
                deferreds.forEach { it.cancel() }
            }
        })
        return this
    }

    fun timeout(action: Action, ms: Long): ActionBuilder {
        targets.addAll(action.targets)
        steps.add(Action.Step {
            action.reset()
            withTimeoutOrNull(ms) { action.runSteps() } ?: action.cancel()
        })
        return this
    }

    fun repeat(action: Action, times: Int): ActionBuilder {
        targets.addAll(action.targets)
        steps.add(Action.Step {
            repeat(times) {
                action.reset()
                action.runSteps()
            }
        })
        return this
    }

    fun repeat(action: Action, times: () -> Int): ActionBuilder {
        targets.addAll(action.targets)
        steps.add(Action.Step {
            repeat(times()) {
                action.reset()
                action.runSteps()
            }
        })
        return this
    }

    fun loop(action: Action, condition: () -> Boolean): ActionBuilder {
        targets.addAll(action.targets)
        steps.add(Action.Step {
            while (condition()) {
                action.reset()
                action.runSteps()
            }
        })
        return this
    }

    fun retry(action: Action, maxAttempts: Int, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        targets.addAll(action.targets)
        steps.add(Action.Step {
            var attempts = 0
            while (attempts < maxAttempts) {
                attempts++
                action.reset()
                action.runSteps()
                if (success()) break
            }
        })
        return this
    }

    fun retry(action: Action, maxAttempts: Int, delayMs: Long, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        require(delayMs >= 0) { "delayMs must be non-negative" }
        targets.addAll(action.targets)
        steps.add(Action.Step {
            var attempts = 0
            while (attempts < maxAttempts) {
                attempts++
                action.reset()
                action.runSteps()
                if (success()) break
                if (attempts < maxAttempts && delayMs > 0) kotlinx.coroutines.delay(delayMs)
            }
        })
        return this
    }

    fun retry(code: Runnable, maxAttempts: Int, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        steps.add(Action.Step {
            var attempts = 0
            while (attempts < maxAttempts) {
                attempts++
                code.run()
                if (success()) break
            }
        })
        return this
    }

    fun retry(code: Runnable, maxAttempts: Int, delayMs: Long, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        require(delayMs >= 0) { "delayMs must be non-negative" }
        steps.add(Action.Step {
            var attempts = 0
            while (attempts < maxAttempts) {
                attempts++
                code.run()
                if (success()) break
                if (attempts < maxAttempts && delayMs > 0) kotlinx.coroutines.delay(delayMs)
            }
        })
        return this
    }

    fun named(name: String): ActionBuilder {
        this.name = name
        return this
    }

    /** Explicitly add a target module (auto-discovery only catches set()-passed states). */
    fun targets(module: Module): ActionBuilder {
        targets.add(module)
        return this
    }

    fun build(): Action = Action(steps.toList(), targets.toSet(), name)
}

inline fun action(
    name: String = "Action",
    block: ActionBuilder.() -> Unit,
): Action = ActionBuilder().apply(block).named(name).build()

private suspend inline fun <R> select(crossinline block: SelectBuilder<R>.() -> Unit): R =
    kotlinx.coroutines.selects.select(block)

private typealias SelectBuilder<R> = kotlinx.coroutines.selects.SelectBuilder<R>
