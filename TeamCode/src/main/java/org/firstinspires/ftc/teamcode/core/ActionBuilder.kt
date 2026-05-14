package org.firstinspires.ftc.teamcode.core

import java.lang.Runnable
import java.util.function.Supplier
import kotlinx.coroutines.async
import kotlinx.coroutines.awaitAll
import kotlinx.coroutines.coroutineScope
import kotlinx.coroutines.withTimeoutOrNull

/**
 * Builder for composing Actions from steps.
 * Use Actions.builder() to create a new builder.
 */
class ActionBuilder {
    private val steps = mutableListOf<Action.Step>()
    private val targets = mutableSetOf<Module>()
    private var name = "Action"

    /** Adds steps to transition to these states. */
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

    /** Deferred state resolution — evaluates suppliers at execution time instead of build time. */
    @SafeVarargs
    fun setLazy(vararg suppliers: Supplier<out State>): ActionBuilder {
        steps.add(Action.Step {
            for (supplier in suppliers) {
                supplier.get().apply()
            }
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

    /** Pauses execution for the given duration. */
    fun delay(ms: Long): ActionBuilder {
        steps.add(Action.Step { kotlinx.coroutines.delay(ms) })
        return this
    }

    fun waitUntil(condition: () -> Boolean): ActionBuilder = waitUntil(condition, Long.MAX_VALUE)

    /** Blocks until the condition becomes true or timeout expires. */
    fun waitUntil(condition: () -> Boolean, timeoutMs: Long): ActionBuilder {
        steps.add(
            Action.Step {
                withTimeoutOrNull(timeoutMs) {
                    while (!condition()) {
                        kotlinx.coroutines.delay(10)
                    }
                }
            },
        )
        return this
    }

    fun waitWhile(condition: () -> Boolean): ActionBuilder = waitUntil({ !condition() })

    fun waitWhile(condition: () -> Boolean, timeoutMs: Long): ActionBuilder =
        waitUntil({ !condition() }, timeoutMs)

    /** Embeds another action's steps and targets. */
    fun action(action: Action): ActionBuilder {
        targets.addAll(action.targets)
        steps.addAll(action.steps)
        return this
    }

    /** Stops executing remaining steps if condition is true. */
    fun stopIf(condition: () -> Boolean): ActionBuilder {
        steps.add(Action.Step { parent ->
            if (condition()) parent.completeEarly()
        })
        return this
    }

    /** Conditionally runs the action. */
    fun ifThen(condition: () -> Boolean, ifTrue: Action): ActionBuilder {
        targets.addAll(ifTrue.targets)

        steps.add(
            Action.Step {
                if (condition()) {
                    ifTrue.runSuspending()
                }
            },
        )
        return this
    }

    /** Conditionally runs one of two actions. */
    fun ifElse(
        condition: () -> Boolean,
        ifTrue: Action,
        ifFalse: Action,
    ): ActionBuilder {
        targets.addAll(ifTrue.targets)
        targets.addAll(ifFalse.targets)

        steps.add(
            Action.Step {
                if (condition()) {
                    ifTrue.runSuspending()
                } else {
                    ifFalse.runSuspending()
                }
            },
        )
        return this
    }

    /** Executes all actions concurrently, waits for all to complete. */
    fun parallel(vararg actions: Action): ActionBuilder {
        actions.forEach { targets.addAll(it.targets) }

        steps.add(
            Action.Step {
                coroutineScope {
                    actions.map { action ->
                        action.reset()
                        async { action.runSteps() }
                    }.awaitAll()
                }
            },
        )
        return this
    }

    /** Executes all actions concurrently, completes when first finishes. */
    fun race(vararg actions: Action): ActionBuilder {
        actions.forEach { targets.addAll(it.targets) }

        steps.add(
            Action.Step {
                coroutineScope {
                    val deferreds = actions.map { action ->
                        action.reset()
                        async { action.runSteps() }
                    }
                    select<Unit> {
                        deferreds.forEach { deferred ->
                            deferred.onAwait { }
                        }
                    }
                    // Cancel remaining coroutines so coroutineScope can return
                    deferreds.forEach { it.cancel() }
                }
            },
        )
        return this
    }

    /** Runs action with a time limit; cancels if timeout exceeded. */
    fun timeout(action: Action, ms: Long): ActionBuilder {
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                action.reset()
                withTimeoutOrNull(ms) {
                    action.runSteps()
                } ?: action.cancel()
            },
        )
        return this
    }

    /** Runs action a fixed number of times. */
    fun repeat(action: Action, times: Int): ActionBuilder {
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                repeat(times) {
                    action.reset()
                    action.runSteps()
                }
            },
        )
        return this
    }

    /** Runs action with a dynamic repetition count. */
    fun repeat(action: Action, times: () -> Int): ActionBuilder {
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                repeat(times()) {
                    action.reset()
                    action.runSteps()
                }
            },
        )
        return this
    }

    /** Runs action repeatedly while condition is true. */
    fun loop(action: Action, condition: () -> Boolean): ActionBuilder {
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                while (condition()) {
                    action.reset()
                    action.runSteps()
                }
            },
        )
        return this
    }

    /** Retries action until success condition met or max attempts reached. */
    fun retry(action: Action, maxAttempts: Int, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                var attempts = 0
                while (attempts < maxAttempts) {
                    attempts++
                    action.reset()
                    action.runSteps()
                    if (success()) break
                }
            },
        )
        return this
    }

    /** Retries action with delay between attempts. */
    fun retry(action: Action, maxAttempts: Int, delayMs: Long, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        require(delayMs >= 0) { "delayMs must be non-negative" }
        targets.addAll(action.targets)

        steps.add(
            Action.Step {
                var attempts = 0
                while (attempts < maxAttempts) {
                    attempts++
                    action.reset()
                    action.runSteps()
                    if (success()) break
                    if (attempts < maxAttempts && delayMs > 0) {
                        kotlinx.coroutines.delay(delayMs)
                    }
                }
            },
        )
        return this
    }

    /** Retries runnable until success condition met or max attempts reached. */
    fun retry(code: Runnable, maxAttempts: Int, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }

        steps.add(
            Action.Step {
                var attempts = 0
                while (attempts < maxAttempts) {
                    attempts++
                    code.run()
                    if (success()) break
                }
            },
        )
        return this
    }

    /** Retries runnable with delay between attempts. */
    fun retry(code: Runnable, maxAttempts: Int, delayMs: Long, success: () -> Boolean): ActionBuilder {
        require(maxAttempts >= 1) { "maxAttempts must be at least 1" }
        require(delayMs >= 0) { "delayMs must be non-negative" }

        steps.add(
            Action.Step {
                var attempts = 0
                while (attempts < maxAttempts) {
                    attempts++
                    code.run()
                    if (success()) break
                    if (attempts < maxAttempts && delayMs > 0) {
                        kotlinx.coroutines.delay(delayMs)
                    }
                }
            },
        )
        return this
    }

    /** Sets the action name for debugging and telemetry. */
    fun named(name: String): ActionBuilder {
        this.name = name
        return this
    }

    /** Explicitly adds a target module for this action. */
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
