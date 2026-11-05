package com.jakubmini.psom.scan

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class ScanStateMachineTest {
    @Test fun happyPath() {
        val m = ScanStateMachine()
        assertTrue(m.press())
        assertEquals(Phase.SCANNING, m.state.phase)
        assertTrue(m.release())
        assertTrue(m.processed(true, null, null))
        assertEquals(Phase.REVIEW, m.state.phase)
        assertTrue(m.exported(true, null, null))
        assertEquals(Phase.EXPORTED, m.state.phase)
    }

    @Test fun releaseWithoutPressIsIgnored() {
        val m = ScanStateMachine()
        assertFalse(m.release())
        assertEquals(Phase.READY, m.state.phase)
        m.press()
        assertFalse(m.press())  // a second finger doesn't restart the scan
    }

    @Test fun failureKeepsTheStageAndCanBeRetried() {
        val m = ScanStateMachine()
        m.press(); m.release()
        m.processed(false, "fusion", "too big")
        assertEquals(ScanState(Phase.FAILED, "fusion", "too big"), m.state)
        assertTrue(m.reprocess())
        assertEquals(Phase.PROCESSING, m.state.phase)
    }

    @Test fun failedExportStaysInReviewWithTheError() {
        val m = ScanStateMachine(ScanState(Phase.REVIEW))
        m.exported(false, "export", "bad shell")
        assertEquals(ScanState(Phase.REVIEW, "export", "bad shell"), m.state)
    }

    @Test fun reopenedProjectsResumeWhereTheyWere() {
        assertEquals(Phase.EXPORTED, ScanStateMachine.resume("exported", null, null).phase)
        assertEquals(Phase.REVIEW, ScanStateMachine.resume("reconstructed", null, null).phase)
        assertEquals(Phase.FAILED, ScanStateMachine.resume("captured", "depth", "no depth").phase)
        assertEquals(Phase.PROCESSING, ScanStateMachine.resume("captured", null, null).phase)
    }
}
