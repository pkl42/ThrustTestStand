#ifndef TEST_PROTOCOL_VALIDATOR_H
#define TEST_PROTOCOL_VALIDATOR_H

#include "protocol/TestProtocol.h"
#include "protocol/ProtocolErrors.h"

struct ProtocolValidationResult
{
    ProtocolError error;
    float totalDurationS;
};

ProtocolValidationResult validateProtocol(const TestProtocol &proto);

#endif // TEST_PROTOCOL_VALIDATOR_H
