#include <assert.h>
#include <stdint.h>

#include "protocol/NkSetTransaction.h"

static void testRejectedSetDoesNotApplyEarlierFields()
{
  int state = 0;
  int validated = 0;
  int applied = 0;
  const bool accepted = runNk4SetTransaction(3, [&](uint8_t index, bool apply) {
    if (!apply)
    {
      validated++;
      return index != 2;
    }
    state = state * 10 + index + 1;
    applied++;
    return true;
  });

  assert(!accepted);
  assert(validated == 3);
  assert(applied == 0);
  assert(state == 0);
}

static void testAcceptedSetAppliesFieldsInOrder()
{
  int state = 0;
  const bool accepted = runNk4SetTransaction(3, [&](uint8_t index, bool apply) {
    if (apply)
    {
      state = state * 10 + index + 1;
    }
    return true;
  });

  assert(accepted);
  assert(state == 123);
}

int main()
{
  testRejectedSetDoesNotApplyEarlierFields();
  testAcceptedSetAppliesFieldsInOrder();
  return 0;
}
