package com.jakubmini.psom.scan

/** READY → SCANNING → PROCESSING → REVIEW → EXPORTED, plus FAILED (with the stage) → retry. */
enum class Phase { READY, SCANNING, PROCESSING, REVIEW, EXPORTED, FAILED }

data class ScanState(val phase: Phase, val stage: String? = null, val error: String? = null)

/** Transitions that don't apply are refused (false) and leave the state alone. */
class ScanStateMachine(initial: ScanState = ScanState(Phase.READY)) {
    var state: ScanState = initial
        private set

    private fun move(from: Set<Phase>, to: ScanState): Boolean {
        if (state.phase !in from) return false
        state = to
        return true
    }

    fun press() = move(setOf(Phase.READY), ScanState(Phase.SCANNING))
    fun release() = move(setOf(Phase.SCANNING), ScanState(Phase.PROCESSING))
    fun processed(ok: Boolean, stage: String?, error: String?) =
        move(setOf(Phase.PROCESSING), if (ok) ScanState(Phase.REVIEW) else ScanState(Phase.FAILED, stage, error))

    /** Export failure keeps you in review with the error shown; success moves on. */
    fun exported(ok: Boolean, stage: String?, error: String?) =
        move(setOf(Phase.REVIEW, Phase.EXPORTED), if (ok) ScanState(Phase.EXPORTED) else ScanState(Phase.REVIEW, stage, error))

    /** Retry after a failure, or re-run after editing the crop: the capture is reused. */
    fun reprocess() = move(setOf(Phase.FAILED, Phase.REVIEW, Phase.EXPORTED), ScanState(Phase.PROCESSING))

    fun reset() {
        state = ScanState(Phase.READY)
    }

    companion object {
        /** Where a reopened project starts, from its project.json. */
        fun resume(status: String, lastErrorStage: String?, lastError: String?): ScanState = when (status) {
            "exported" -> ScanState(Phase.EXPORTED)
            "reconstructed" -> ScanState(Phase.REVIEW, lastErrorStage, lastError)
            else -> if (lastError != null) ScanState(Phase.FAILED, lastErrorStage, lastError) else ScanState(Phase.PROCESSING)
        }
    }
}
