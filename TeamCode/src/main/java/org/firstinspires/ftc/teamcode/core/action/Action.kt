package org.firstinspires.ftc.teamcode.core.action

import org.firstinspires.ftc.teamcode.core.Module
import org.firstinspires.ftc.teamcode.core.state.State
import java.lang.Runnable
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancelChildren
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

class Action internal constructor(
    internal val steps: List<Step>,
    private val targetModules: Set<Module>,
    private val actionName: String,
) {
    private var job: Job? = null
    private val stepIndex = AtomicInteger(0)
    private val cancelled = AtomicBoolean(false)
    private val completedEarly = AtomicBoolean(false)

    /** Schedule async; actions targeting the same modules auto-cancel each other. */
    fun run() {
        Actions.run(this)
    }

    /** Block the caller until this action completes. Don't call from the OpMode loop thread. */
    fun runBlocking() {
        runBlocking { runSuspending() }
    }

    suspend fun runSuspending() {
        reset()
        runSteps()
    }

    fun cancel() {
        cancelled.set(true)
        job?.cancel()
    }

    val isRunning: Boolean
        get() = job?.isActive == true

    val isComplete: Boolean
        get() = !isRunning && !cancelled.get() && (completedEarly.get() || stepIndex.get() >= steps.size)

    val isCancelled: Boolean
        get() = cancelled.get()

    val progress: Float
        get() = if (steps.isEmpty()) 1f else stepIndex.get().toFloat() / steps.size

    val targets: Set<Module>
        get() = targetModules

    val name: String
        get() = actionName

    /** Stop after the current step finishes, without flagging the action as cancelled. */
    fun completeEarly() {
        completedEarly.set(true)
    }

    internal fun reset() {
        stepIndex.set(0)
        cancelled.set(false)
        completedEarly.set(false)
        job = null
    }

    internal fun setJob(job: Job) {
        this.job = job
    }

    internal suspend fun runSteps() {
        stepIndex.set(0)

        for ((index, step) in steps.withIndex()) {
            if (cancelled.get() || completedEarly.get()) break
            stepIndex.set(index)
            try {
                step.run(this)
            } catch (e: CancellationException) {
                cancelled.set(true)
                throw e
            } catch (e: Exception) {
                val targetInfo = if (targetModules.isNotEmpty()) {
                    "targets: [" + targetModules.joinToString(", ") { it.name } + "]"
                } else {
                    "no targets"
                }
                System.err.println("[ERROR] Action '$actionName' failed at step ${index + 1}/${steps.size} ($targetInfo): ${e.message}")
                e.printStackTrace()
                break
            }
        }

        if (!cancelled.get()) {
            stepIndex.set(steps.size)
        }
    }

    infix fun then(other: Action): Action = Actions.sequence(this, other)

    infix fun with(other: Action): Action = Actions.parallel(this, other)

    fun timeout(ms: Long): Action = Actions.timeout(this, ms)

    fun named(newName: String): Action = Action(steps, targetModules, newName)

    override fun toString(): String = "$actionName(${steps.size} steps)"

    fun interface Step {
        suspend fun run(parent: Action)
    }
}

object Actions {
    private val active = ConcurrentHashMap<Action, Job>()
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    @JvmStatic
    fun set(vararg states: State): Action {
        val b = ActionBuilder()
        states.forEach { b.set(it) }
        return b.build()
    }

    /** Resolve states at execution time rather than build time. */
    @SafeVarargs
    @JvmStatic
    fun setLazy(vararg suppliers: java.util.function.Supplier<out State>): Action =
        ActionBuilder().setLazy(*suppliers).build()

    @JvmStatic
    fun run(code: Runnable): Action = ActionBuilder().run(code).build()

    @JvmStatic
    fun run(code: () -> Unit): Action = ActionBuilder().run(code).build()

    @JvmStatic
    fun runSuspending(code: suspend () -> Unit): Action = ActionBuilder().runSuspending(code).build()

    @JvmStatic
    fun delay(ms: Long): Action = ActionBuilder().delay(ms).build()

    @JvmStatic
    fun noop(): Action = ActionBuilder().build()

    @JvmStatic
    fun waitUntil(condition: () -> Boolean): Action = ActionBuilder().waitUntil(condition).build()

    @JvmStatic
    fun waitUntil(condition: () -> Boolean, timeoutMs: Long): Action =
        ActionBuilder().waitUntil(condition, timeoutMs).build()

    @JvmStatic
    fun sequence(vararg actions: Action): Action {
        val b = ActionBuilder()
        actions.forEach { b.action(it) }
        return b.named("Sequence").build()
    }

    @JvmStatic
    fun parallel(vararg actions: Action): Action =
        ActionBuilder().parallel(*actions).named("Parallel").build()

    @JvmStatic
    fun race(vararg actions: Action): Action =
        ActionBuilder().race(*actions).named("Race").build()

    @JvmStatic
    fun timeout(action: Action, ms: Long): Action =
        ActionBuilder().timeout(action, ms).named("Timeout").build()

    @JvmStatic
    fun repeat(action: Action, times: Int): Action =
        ActionBuilder().repeat(action, times).named("Repeat").build()

    @JvmStatic
    fun loop(action: Action, condition: () -> Boolean): Action =
        ActionBuilder().loop(action, condition).named("Loop").build()

    @JvmStatic
    fun ifThen(condition: () -> Boolean, ifTrue: Action): Action =
        ActionBuilder().ifThen(condition, ifTrue).named("IfThen").build()

    @JvmStatic
    fun ifElse(condition: () -> Boolean, ifTrue: Action, ifFalse: Action): Action =
        ActionBuilder().ifElse(condition, ifTrue, ifFalse).named("IfElse").build()

    @JvmStatic
    fun retry(action: Action, maxAttempts: Int, success: () -> Boolean): Action =
        ActionBuilder().retry(action, maxAttempts, success).named("Retry").build()

    @JvmStatic
    fun retry(action: Action, maxAttempts: Int, delayMs: Long, success: () -> Boolean): Action =
        ActionBuilder().retry(action, maxAttempts, delayMs, success).named("Retry").build()

    @JvmStatic
    fun retry(code: Runnable, maxAttempts: Int, success: () -> Boolean): Action =
        ActionBuilder().retry(code, maxAttempts, success).named("Retry").build()

    @JvmStatic
    fun retry(code: Runnable, maxAttempts: Int, delayMs: Long, success: () -> Boolean): Action =
        ActionBuilder().retry(code, maxAttempts, delayMs, success).named("Retry").build()

    @JvmStatic
    fun builder(): ActionBuilder = ActionBuilder()

    internal fun run(action: Action) {
        active.keys.filter { conflicts(it, action) }.forEach {
            it.cancel()
            active.remove(it)
        }

        active[action]?.let {
            action.cancel()
            active.remove(action)
        }

        action.reset()
        val job = scope.launch {
            val myJob = coroutineContext[Job]!!
            try {
                action.runSteps()
            } finally {
                // Only remove if we're still the active job — guard against a re-launch that
                // replaced us between launch and finally.
                active.remove(action, myJob)
            }
        }

        action.setJob(job)
        active[action] = job
    }

    private fun conflicts(a: Action, b: Action): Boolean =
        a.targets.any { it in b.targets }

    @JvmStatic
    fun cancelAll() {
        active.keys.forEach { it.cancel() }
        active.clear()
    }

    @JvmStatic
    fun cancelFor(vararg modules: Module) {
        active.keys.filter { action -> modules.any { it in action.targets } }.forEach {
            it.cancel()
            active.remove(it)
        }
    }

    @JvmStatic
    fun hasActive(): Boolean = active.isNotEmpty()

    @JvmStatic
    fun activeCount(): Int = active.size

    @JvmStatic
    fun isModuleActive(module: Module): Boolean =
        active.keys.any { module in it.targets }

    @JvmStatic
    fun getActive(): List<Action> = active.keys.toList()

    @JvmStatic
    fun shutdown() {
        cancelAll()
        scope.coroutineContext.cancelChildren()
    }

    @JvmStatic
    fun reset() {
        cancelAll()
    }
}
