/*
* Written 2008, 2009, 2010 by <mgix@mgix.com>
* This code is in the public domain
* See http://www.mgix.com/snippets/?LockFree for details
*
* A lock-free LIFO stack for x86 CPUs
* Requires a CPU that supports cmpxchg(8|16)
*/
#ifndef __LIFO_H__
#define __LIFO_H__

namespace nonlocking{

#include <assert.h>
#include <inttypes.h>

    struct Lifo
    {

        struct Node;
        Node *volatile head;
        uintptr_t volatile sync;
        struct Node { Node *volatile next; };

        inline Lifo();
        inline void push(Node *node);
        inline uint32_t tryPush(Node *node);
        inline Node *tryPop(uint32_t &success);
        inline Node *pop();

    } __attribute__((aligned(2*sizeof(uintptr_t)), packed));

    inline Lifo::Lifo()
    {
#if defined(_DEBUG)
            const size_t size = sizeof(uintptr_t);
            const size_t mask = 2*sizeof(uintptr_t)-1;
            uintptr_t p0 = (uintptr_t)this; assert(0==(p0&mask));
            uintptr_t p1 = (uintptr_t)&(this->head); assert(p0==p1);
            uintptr_t p2 = (uintptr_t)&(this->sync); assert(p2-p0==size);
            assert(sizeof(Lifo::sync)==size);
            assert(sizeof(Lifo::head)==size);
#endif
        sync = 0;
        head = 0;
    }

#if defined(__i386__)

#if defined(__PIC__)

            // In 32bit PIC mode, ebx in the clobber list makes gcc sad, we do the legwork
#define PUSH_EBX "pushl %%ebx \n"
#define LOAD_EBX "movl " NODE_INDEX ", %%ebx # ebx = node \n"
#define POP_EBX "popl %%ebx \n"
#define MAP_EBX "r" (node)
#define CLOB_EBX ""

#else

            // In 32bit non-PIC mode, ebx is just another register, gcc does the legwork
#define PUSH_EBX "\n"
#define LOAD_EBX "\n"
#define POP_EBX "\n"
#define MAP_EBX "b" (node)
#define CLOB_EBX "ebx",

#endif

        // Core of push code, used by both push and tryPush
#define PUSH_CORE \
asm volatile( \
PUSH_EBX \
LOAD_EBX \
"movl (%%edi), %%eax # eax = lifo->head \n" \
"movl 4(%%edi), %%edx # edx = lifo->sync \n" \
"0: \n" \
"movl %%eax, (%%ebx) # node->next = lifo->head \n" \
"movl %%edx, %%ecx # ecx = lifo->sync \n" \
"inc %%ecx # ecx = lifo->sync+1 \n" \
"lock cmpxchg8b (%%edi) # lifo->content = newContent iff lifo->content unchanged \n" \
PUSH_FINISH \
POP_EBX \
: PUSH_RESULT \
: "D" (this), \
MAP_EBX \
: "memory", \
CLOB_EAX \
"ecx", \
"edx", \
"cc" \
); \

            // Push a node in a busy loop until success
            inline void Lifo::push(
                Lifo::Node *node
            )
            {
#define PUSH_FINISH "jnz 0b # try again if missed \n"
#define NODE_INDEX "%1"
#define CLOB_EAX "eax",
#define PUSH_RESULT
                    PUSH_CORE
#undef PUSH_FINISH
#undef PUSH_RESULT
#undef NODE_INDEX
#undef CLOB_EAX
            }

            // Try to push a node, return 1 if success 0 if failed
            inline uint32_t Lifo::tryPush(
                Lifo::Node *node
            )
            {
                uint32_t result;
#define PUSH_FINISH \
"setz %%al # al = success \n" \
"movzbl %%al, %%eax # eax = zeroExtend(al) \n"
#define PUSH_RESULT "=a" (result)
#define NODE_INDEX "%2"
#define CLOB_EAX
                    PUSH_CORE
#undef PUSH_FINISH
#undef PUSH_RESULT
#undef NODE_INDEX
#undef CLOB_EAX
                return result;
            }

#undef PUSH_CORE

#define POP_CORE \
asm volatile( \
PUSH_EBX \
"movl (%%esi), %%eax # eax = lifo->head \n" \
"movl 4(%%esi), %%edx # edx = lifo->sync \n" \
"0: \n" \
"testl %%eax , %%eax # is lifo->head 0 ? \n" \
"jz 1f # yes -> bail \n" \
"movl (%%eax), %%ebx # ebx = lifo->head->next \n" \
"movl %%edx, %%ecx # ecx = lifo->sync \n" \
"inc %%ecx # ecx = lifo->sync+1 \n" \
"lock cmpxchg8b (%%esi) # lifo->content = newContent iff lifo->content is unchanged \n" \
POP_FINISH \
"1: \n" \
POP_EBX \
: "=a" (popped) \
POP_RESULT \
: "S" (this) \
: "memory", \
CLOB_EBX \
"ecx", \
"edx", \
"cc" \
); \

            inline Lifo::Node *Lifo::pop()
            {
                Node *popped;
#define POP_FINISH "jnz 0b # try again if missed \n"
#define POP_RESULT
                    POP_CORE
#undef POP_FINISH
#undef POP_RESULT
                return popped;
            }

            Lifo::Node *Lifo::tryPop(
                uint32_t &success
            )
            {
                Node *popped;
#define POP_FINISH \
"setz %%dl # dl = success \n" \
"movzbl %%dl, %%edi # edi = zeroExtend(dl) \n"
#define POP_RESULT ,"=D" (success)
                    POP_CORE
#undef POP_FINISH
#undef POP_RESULT
                return popped;
            }

#undef POP_CORE

#undef CLOB_EBX
#undef PUSH_EBX
#undef LOAD_EBX
#undef MAP_EBX
#undef POP_EBX

#endif // defined(__i386__)

#if defined(__x86_64__)

#define PUSH_CORE \
asm volatile( \
"movq (%%rdi), %%rax # rax = lifo->head \n" \
"movq 8(%%rdi), %%rdx # rdx = lifo->sync \n" \
"0: \n" \
"movq %%rax, (%%rbx) # node->next = lifo->head \n" \
"movq %%rdx, %%rcx # rcx = lifo->sync \n" \
"inc %%rcx # rcx = lifo->sync+1 \n" \
"lock cmpxchg16b (%%rdi) # lifo->content = newContent iff lifo->content unchanged \n" \
PUSH_FINISH \
: PUSH_RESULT \
: "D" (this), \
"b" (node) \
: "memory", \
CLOB_RAX \
"rcx", \
"rdx", \
"cc" \
); \

            inline void Lifo::push(
                Lifo::Node *node
            )
            {
#define PUSH_RESULT
#define CLOB_RAX "rax",
#define PUSH_FINISH "jnz 0b # try again if missed \n"
                    PUSH_CORE
#undef PUSH_FINISH
#undef PUSH_RESULT
#undef CLOB_RAX
            }

            inline uint32_t Lifo::tryPush(
                Node *node
            )
            {
                uint32_t result;
#define CLOB_RAX
#define PUSH_FINISH \
"setz %%al # al = success \n" \
"movzbl %%al, %%eax # eax = zeroExtend(al) \n"
#define PUSH_RESULT "=a" (result)
                    PUSH_CORE
#undef PUSH_FINISH
#undef PUSH_RESULT
#undef CLOB_RAX
                return result;
            }

#undef PUSH_CORE

#define POP_CORE \
asm volatile( \
"0: \n" \
"movq (%%rdi), %%rax # rax = lifo->head \n" \
"movq 8(%%rdi), %%rdx # rdx = lifo->sync \n" \
"testq %%rax , %%rax # is lifo->head 0 ? \n" \
"jz 1f # yes -> bail \n" \
"movq (%%rax), %%rbx # rbx = lifo->head->next \n" \
"leaq 1(%%rdx), %%rcx # rcx = lifo->sync + 1 \n" \
"lock cmpxchg16b (%%rdi) # lifo->head = next iff lifo->content is unchanged \n" \
POP_FINISH \
"1: \n" \
: "=a" (popped) \
POP_RESULT \
: "D" (this) \
: "memory", \
"rbx", \
"rcx", \
CLOB_RDX \
"cc" \
); \

            inline Lifo::Node *Lifo::pop()
            {
                Node *popped;
#define POP_RESULT
#define CLOB_RDX "rdx",
#define POP_FINISH "jnz 0b # try again if missed \n"
                    POP_CORE
#undef POP_FINISH
#undef POP_RESULT
#undef CLOB_RDX
                return popped;
            }

            inline Lifo::Node *Lifo::tryPop(
                uint32_t &success
            )
            {
                Node *popped;
#define CLOB_RDX
#define POP_FINISH \
"setz %%dl # dl = success \n" \
"movzbl %%dl, %%edi # edi = zeroExtend(dl) \n"
#define POP_RESULT ,"=D" (success)
                    POP_CORE
#undef POP_RESULT
#undef POP_FINISH
#undef CLOB_RDX
                return popped;
            }

#undef POP_CORE

#endif // defined(__x86_64__)
}
#endif // __LIFO_H__

