// Class for handling the encoding of PPS status

#ifndef PPSPROC_H
#define PPSPROC_H

class CPPSProc
{
public:
    CPPSProc() : m_pPPS(nullptr) {;}
    CPPSProc(unsigned char* pPPS) {m_pPPS = pPPS;}

    unsigned char* m_pPPS;

    enum PPSdecode
    {
        // 0 / 1
        PPS_DISABLE_BIT			= 0x1,      // 1PPS is enabled / 1PPS is disabled
        PPS_EDGE_BIT			= 0x2,      // 1PPS acts on rising edge	/ 1PPS acts on falling edge
        PPS_ACKNOWLEDGE_BIT		= 0x4,      // TEM has not received 1PPS / TEM has received 1PPS
        PPS_PERIOD_ERROR_BIT	= 0x8,      // PPS period matches TEM clock / PPS period does not match TEM clock
        PPS_USE_PC_TIME			= 0x10      // Ignore TEM time altogether and use the PC clock instead
    };

    bool PPSEnabled()			const	{return (((*m_pPPS) & PPS_DISABLE_BIT) == 0);}
    void EnablePPS(bool enable)			{if (enable) (*m_pPPS) &= ~PPS_DISABLE_BIT; else (*m_pPPS) |= PPS_DISABLE_BIT;}
    bool PPSRisingEdge()		const	{return (((*m_pPPS) & PPS_EDGE_BIT) == 0);}
    void SetPPSEdge(bool rising)		{if (rising) (*m_pPPS)	&= ~PPS_EDGE_BIT; else (*m_pPPS) |= PPS_EDGE_BIT;}
    bool UsePCTime()			const	{return (((*m_pPPS) & PPS_USE_PC_TIME) != 0);}							// Warning; the sense of this has got inverted at some point; it returns true if the sonar is set to sonar clock
    void SetUsePCTime(bool usePCTime)	{ if (usePCTime) (*m_pPPS) |= PPS_USE_PC_TIME; else (*m_pPPS)	&= ~PPS_USE_PC_TIME;}
    bool PPSAcknowledged()		const	{return (((*m_pPPS) & PPS_ACKNOWLEDGE_BIT) != 0);}
    void SetPPSAcknowledged(bool received) {if (received) (*m_pPPS) |= PPS_ACKNOWLEDGE_BIT; else (*m_pPPS) &= ~PPS_ACKNOWLEDGE_BIT;}
    bool PPSPeriodError()		const	{return (((*m_pPPS) & PPS_PERIOD_ERROR_BIT) != 0);}
    void SetPPSPeriodError(bool error)	{if (error) (*m_pPPS) |= PPS_PERIOD_ERROR_BIT; else (*m_pPPS) &= ~PPS_PERIOD_ERROR_BIT;}
};

#endif  // PPSPROC_H
